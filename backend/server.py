import os
import glob
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
from dotenv import load_dotenv 

load_dotenv() 


api_key = None


try:
    from config_keys import GOOGLE_API_KEY
    api_key = GOOGLE_API_KEY
    print(f"Loaded Google Key from config_keys.py")
except ImportError:
    pass


if not api_key:
    env_key = os.getenv("GOOGLE_API_KEY")
    if env_key:
        api_key = env_key.strip().replace('"', '').replace("'", "")
        print(f"Loaded Google Key from Environment")

if not api_key:
    print("CRITICAL ERROR: Google API Key kahin nahi mili!")


COLLECTION_NAME = "physical_ai_book"
DOCS_DIR = "../docs"


qdrant_url = os.getenv("QDRANT_URL")
qdrant_key = os.getenv("QDRANT_API_KEY")


if qdrant_url: qdrant_url = qdrant_url.strip().replace('"', '').replace("'", "")
if qdrant_key: qdrant_key = qdrant_key.strip().replace('"', '').replace("'", "")

if qdrant_url and qdrant_key and len(qdrant_url) > 5:
    print(f"Connecting to Qdrant Cloud... ({qdrant_url[:20]}...)")
    try:
        qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_key)
        
        qdrant.get_collections()
        print(" Connected to Cloud Database Successfully!")
    except Exception as e:
        print(f" CLOUD CONNECTION FAILED: {e}")
        print(" System stopping because Cloud Database is required.")
        raise e
else:
    print(" ERROR: QDRANT_URL or QDRANT_API_KEY missing in .env")
    print(" System stopping because Cloud Database is required.")
    raise ValueError("Cloud Credentials Missing")

print(" Loading Embedding Model...")
encoder = SentenceTransformer('all-MiniLM-L6-v2')


def ingest_book():
    print("Checking Cloud Database for Book...")
    try:
        collections = qdrant.get_collections().collections
        for c in collections:
            if c.name == COLLECTION_NAME:
                print("Book already exists in Cloud Database.")
                return
    except Exception as e:
        print(f"Error checking collections: {e}")

    print(f"Database Empty! Uploading Book from '{DOCS_DIR}' to Cloud...")
    
    if not os.path.exists(DOCS_DIR):
        print(f"Docs folder missing at: {DOCS_DIR}")
        return

    documents = []
    for filepath in glob.glob(os.path.join(DOCS_DIR, "**/*.md"), recursive=True):
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                content = f.read()
                chunks = content.split("\n\n")
                for i, chunk in enumerate(chunks):
                    if len(chunk) > 50:
                        documents.append({"text": chunk, "id": i})
            print(f"   -> Read file: {filepath}")
        except Exception as e:
            print(f"Could not read {filepath}: {e}")

    if not documents:
        print("No text found in docs to upload!")
        return
    
    print("Uploading vectors to Qdrant Cloud (Please wait)...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE),
    )

    points = []
    for idx, doc in enumerate(documents):
        vector = encoder.encode(doc["text"]).tolist()
        points.append(models.PointStruct(id=idx, vector=vector, payload={"text": doc["text"]}))

    qdrant.upsert(collection_name=COLLECTION_NAME, points=points)
    print("BOOK UPLOADED TO CLOUD SUCCESSFULLY!")


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
    print(f" User asked: {req.question}")
    
    if not api_key:
        return {"answer": "Server Error: Google API Key missing."}
    
    try:
        genai.configure(api_key=api_key)
        local_model = genai.GenerativeModel('gemini-2.5-flash')
    except Exception as e:
        return {"answer": f"Config Error: {str(e)}"}

    context = ""
    if req.selected_text:
        context += f"SELECTED:\n{req.selected_text}\n\n"

    try:
        
        query_vector = encoder.encode(req.question).tolist()
        hits = qdrant.search(collection_name=COLLECTION_NAME, query_vector=query_vector, limit=3)

        if hits:
            book_context = "\n".join([hit.payload['text'] for hit in hits])
            context += f"BOOK CONTEXT:\n{book_context}"
        
        prompt = f"""You are an AI Tutor. Answer based on context.
        Context: {context}
        Question: {req.question}"""
        
        response = local_model.generate_content(prompt)
        print(" Answer Sent!")
        return {"answer": response.text}

    except Exception as e:
        print(f" Error: {e}")
        return {"answer": f"Error: {str(e)}"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)