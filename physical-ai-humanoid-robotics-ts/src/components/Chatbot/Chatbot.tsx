import React, { useState, useEffect, useRef } from 'react';
import './Chatbot.css';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

const ChatbotComponent: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Check for selected text when user makes a selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 10) { // Only consider meaningful selections
        setSelectedText(selectedText);
      } else {
        setSelectedText(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request payload
      const requestBody: any = {
        message: inputValue,
        history: messages.map(msg => ({
          role: msg.role,
          content: msg.content
        }))
      };

      // Include selected text if available
      if (selectedText) {
        requestBody.selected_text = selectedText;
      }

      // Call the backend API
      const response = await fetch('https://ai-native-book-with-rag-chat-bot-production.up.railway.app/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant message
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.response,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);

      // Clear selected text after use
      if (selectedText) {
        setSelectedText(null);
        window.getSelection()?.empty(); // Clear the selection
      }
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Floating chat button */}
      <button
        className={`chatbot-button ${isOpen ? 'hidden' : ''}`}
        onClick={toggleChat}
        aria-label="Open chat"
      >
        💬
      </button>

      {/* Chat container */}
      <div className={`chatbot-container ${isOpen ? 'open' : ''}`}>
        <div className="chatbot-header">
          <h3>Physical AI & Humanoid Robotics Assistant</h3>
          <button
            className="chatbot-close"
            onClick={toggleChat}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>

        {selectedText && (
          <div className="selected-text-indicator">
            <strong>Using selected text:</strong> {selectedText.substring(0, 100)}...
          </div>
        )}

        <div className="chatbot-messages">
          {messages.length === 0 ? (
            <div className="chatbot-welcome">
              <p>Hello! I'm your Physical AI & Humanoid Robotics assistant.</p>
              <p>You can ask me questions about the book content.</p>
              <p>Select text on the page and ask questions about it!</p>
            </div>
          ) : (
            messages.map((message) => (
              <div
                key={message.id}
                className={`chatbot-message ${message.role}`}
              >
                <div className="message-content">{message.content}</div>
                <div className="message-timestamp">
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))
          )}
          {isLoading && (
            <div className="chatbot-message assistant">
              <div className="message-content">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <form onSubmit={handleSubmit} className="chatbot-input-form">
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder={selectedText ? "Ask about selected text..." : "Ask a question about the book..."}
            disabled={isLoading}
          />
          <button type="submit" disabled={isLoading || !inputValue.trim()}>
            Send
          </button>
        </form>
      </div>
    </>
  );
};

const Chatbot: React.FC = () => {
  return (
    <BrowserOnly fallback={<div></div>}>
      {() => <ChatbotComponent />}
    </BrowserOnly>
  );
};

export default Chatbot;