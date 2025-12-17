import os
import glob
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


load_dotenv()

api_key = None
try:
    from config_keys import GOOGLE_API_KEY
    api_key = GOOGLE_API_KEY
except ImportError:
    pass

if not api_key:
    env_key = os.getenv("GOOGLE_API_KEY")
    if env_key:
        api_key = env_key.strip().replace('"', '').replace("'", "")

if not api_key:
    print("CRITICAL ERROR: Google API Key Missing!")
else:
    genai.configure(api_key=api_key)


qdrant_url = os.getenv("QDRANT_URL")
qdrant_key = os.getenv("QDRANT_API_KEY")

if qdrant_url: qdrant_url = qdrant_url.strip().replace('"', '').replace("'", "")
if qdrant_key: qdrant_key = qdrant_key.strip().replace('"', '').replace("'", "")


def get_embedding(text):
    try:
        result = genai.embed_content(
            model="models/text-embedding-004",
            content=text,
            task_type="retrieval_document",
            title="Embedding"
        )
        return result['embedding']
    except Exception as e:
        print(f"Embedding Error: {e}")
        return []


try:
    if qdrant_url and qdrant_key:
        qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_key)
        print("Connected to Qdrant Cloud")
    else:
        qdrant = QdrantClient(":memory:")
        print("Using Memory Database")
except Exception as e:
    print(f"DB Connection Error: {e}")
    qdrant = QdrantClient(":memory:")

COLLECTION_NAME = "physical_ai_book_v2" 


def ingest_book():
    print("Checking Database...")
    try:
        collections = qdrant.get_collections().collections
        for c in collections:
            if c.name == COLLECTION_NAME:
                print("Book is ready in Cloud.")
                return
    except:
        pass

    print("Uploading Book...")
    documents = []
    
   
    search_path = os.path.join(os.getcwd(), "../docs")
    if not os.path.exists(search_path):
        search_path = "../docs" 
    
    for filepath in glob.glob(os.path.join(search_path, "**/*.md"), recursive=True):
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                content = f.read()
                chunks = content.split("\n\n")
                for i, chunk in enumerate(chunks):
                    if len(chunk) > 50:
                        documents.append({"text": chunk, "id": i})
        except Exception as e:
            pass

    if not documents:
        print("No documents found")
        return

    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
    )

    points = []
    for idx, doc in enumerate(documents):
        vector = get_embedding(doc["text"])
        if vector:
            points.append(models.PointStruct(id=idx, vector=vector, payload={"text": doc["text"]}))

    if points:
        qdrant.upsert(collection_name=COLLECTION_NAME, points=points)
        print(f"Uploaded {len(points)} chunks!")


@asynccontextmanager
async def lifespan(app: FastAPI):
    ingest_book()
    yield

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    question: str
    selected_text: str = "" 

@app.post("/chat")
async def chat(req: ChatRequest):
    if not api_key: return {"answer": "API Key Missing"}

    context = ""
    
    if req.selected_text:
        context = f"USER SELECTED TEXT FROM BOOK:\n{req.selected_text}\n\n(Focus strictly on explaining this text)"
    
    
    else:
        query_vector = get_embedding(req.question)
        if query_vector:
            try:
                hits = qdrant.search(
                    collection_name=COLLECTION_NAME,
                    query_vector=query_vector,
                    limit=3
                )
                if hits:
                    book_content = "\n".join([hit.payload['text'] for hit in hits])
                    context = f"BOOK CONTEXT:\n{book_content}"
            except Exception as e:
                print(f"Search Error: {e}")

    prompt = f"""You are an intelligent AI Assistant explaining a book on Physical AI and Robotics.

    YOUR INSTRUCTIONS:
    1. **Context is King:** Always prioritize the provided 'BOOK CONTEXT' or 'USER SELECTED TEXT' to answer.
    2. **Be Conversational:** If the user says "Hi", "Hello", or asks "How are you", answer politely and friendly.
    3. **Fill Gaps:** If the user asks a question about Robotics/AI that isn't perfectly found in the context, USE YOUR OWN KNOWLEDGE to explain it clearly, but try to relate it back to the theme of Physical AI.
    4. **Explain Selections:** If 'USER SELECTED TEXT' is provided, explain that specific text clearly to the user.

    CONTEXT PROVIDED:
    {context}

    USER QUESTION: {req.question}
    """
    
    try:
        model = genai.GenerativeModel('gemini-2.5-flash')
        response = model.generate_content(prompt)
        return {"answer": response.text}
    except Exception as e:
        return {"answer": f"Error: {str(e)}"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)