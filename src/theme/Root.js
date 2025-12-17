import React, { useState, useRef, useEffect } from 'react';

const styles = `
  .chatbot-container {
    position: fixed;
    bottom: 20px;
    right: 20px;
    z-index: 9999;
    font-family: 'Inter', sans-serif;
  }

  /* Toggle Button (Floating Icon) */
  .chatbot-toggle {
    background: linear-gradient(135deg, #007bff, #0056b3);
    color: white;
    border: none;
    border-radius: 50%;
    width: 60px;
    height: 60px;
    cursor: pointer;
    box-shadow: 0 4px 15px rgba(0, 123, 255, 0.4);
    display: flex;
    align-items: center;
    justify-content: center;
    transition: all 0.3s ease;
  }
  
  .chatbot-toggle:hover {
    transform: scale(1.1);
    box-shadow: 0 6px 20px rgba(0, 123, 255, 0.6);
  }

  /* Chat Window (The Box) */
  .chatbot-window {
    position: absolute;
    bottom: 80px;
    right: 0;
    width: 350px;
    height: 500px;
    background-color: #1e1e1e; /* Dark Background */
    border-radius: 20px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
    display: flex;
    flex-direction: column;
    overflow: hidden;
    border: 1px solid #333;
    animation: slideUp 0.3s ease-out;
  }

  @keyframes slideUp {
    from { opacity: 0; transform: translateY(20px); }
    to { opacity: 1; transform: translateY(0); }
  }

  /* Header */
  .chatbot-header {
    background: #252525;
    padding: 15px;
    color: white;
    font-weight: bold;
    display: flex;
    justify-content: space-between;
    align-items: center;
    border-bottom: 1px solid #333;
  }

  .chatbot-header h3 { margin: 0; font-size: 16px; }
  .close-btn { background: none; border: none; color: #aaa; cursor: pointer; font-size: 20px; }
  .close-btn:hover { color: white; }

  /* Messages Area */
  .chatbot-messages {
    flex: 1;
    padding: 15px;
    overflow-y: auto;
    background-color: #121212; /* Deep Dark */
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  /* Bubbles */
  .message {
    max-width: 80%;
    padding: 10px 14px;
    border-radius: 12px;
    font-size: 14px;
    line-height: 1.4;
  }

  .user-msg {
    align-self: flex-end;
    background: #007bff;
    color: white;
    border-bottom-right-radius: 2px;
  }

  .bot-msg {
    align-self: flex-start;
    background: #2d2d2d;
    color: #e0e0e0;
    border-bottom-left-radius: 2px;
    border: 1px solid #444;
  }

  /* Input Area */
  .chatbot-input-area {
    padding: 15px;
    background: #252525;
    display: flex;
    gap: 10px;
    border-top: 1px solid #333;
  }

  .chatbot-input {
    flex: 1;
    padding: 10px;
    border-radius: 20px;
    border: 1px solid #444;
    background: #1e1e1e;
    color: white;
    outline: none;
  }
  
  .chatbot-input:focus { border-color: #007bff; }

  .send-btn {
    background: #007bff;
    color: white;
    border: none;
    border-radius: 50%;
    width: 40px;
    height: 40px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: background 0.2s;
  }

  .send-btn:hover { background: #0056b3; }
  .send-btn:disabled { background: #444; cursor: not-allowed; }
`;

export default function Root({ children }) {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { text: "Hello! I am your AI Book Assistant. Ask me anything!", sender: "bot" }
  ]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);
  
  const messagesEndRef = useRef(null);
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  const handleSend = async () => {
    if (!input.trim()) return;

    const userMessage = input;
    setMessages(prev => [...prev, { text: userMessage, sender: "user" }]);
    setInput("");
    setLoading(true);

    try {
      const response = await fetch("https://hackathon-project.vercel.app/chat", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ question: userMessage }),
      });

      const data = await response.json();
      setMessages(prev => [...prev, { text: data.answer, sender: "bot" }]);
    } catch (error) {
      setMessages(prev => [...prev, { text: "Server Error. Check backend.", sender: "bot" }]);
    }
    setLoading(false);
  };

  return (
    <>
      <style>{styles}</style>
      {children}
      
      <div className="chatbot-container">
        {isOpen && (
          <div className="chatbot-window">
            <div className="chatbot-header">
              <h3>Book Assistant</h3>
              <button className="close-btn" onClick={() => setIsOpen(false)}>×</button>
            </div>

            <div className="chatbot-messages">
              {messages.map((msg, index) => (
                <div key={index} className={`message ${msg.sender === "user" ? "user-msg" : "bot-msg"}`}>
                  {msg.text}
                </div>
              ))}
              {loading && <div className="message bot-msg">Thinking... ⏳</div>}
              <div ref={messagesEndRef} />
            </div>

            <div className="chatbot-input-area">
              <input 
                className="chatbot-input"
                placeholder="Ask a question..." 
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && handleSend()}
              />
              <button className="send-btn" onClick={handleSend} disabled={loading}>➤</button>
            </div>
          </div>
        )}

        {!isOpen && (
          <button className="chatbot-toggle" onClick={() => setIsOpen(true)}>
            💬
          </button>
        )}
      </div>
    </>
  );
}