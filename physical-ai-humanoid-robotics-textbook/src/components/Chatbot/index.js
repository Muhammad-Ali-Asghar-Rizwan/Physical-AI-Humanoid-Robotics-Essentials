import React, { useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

const CHAT_HISTORY_KEY = 'chatbot_history';

function Chatbot({ selectedTextFromPage }) {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [currentSelectedText, setCurrentSelectedText] = useState('');

  // Load messages from local storage on component mount
  useEffect(() => {
    const storedMessages = localStorage.getItem(CHAT_HISTORY_KEY);
    if (storedMessages) {
      setMessages(JSON.parse(storedMessages));
    }
  }, []);

  // Save messages to local storage whenever messages state changes
  useEffect(() => {
    localStorage.setItem(CHAT_HISTORY_KEY, JSON.stringify(messages));
  }, [messages]);

  useEffect(() => {
    setCurrentSelectedText(selectedTextFromPage);
  }, [selectedTextFromPage]);

  const { siteConfig } = useDocusaurusContext();

  const BACKEND_API_URL = (siteConfig && siteConfig.customFields && siteConfig.customFields.backendApiUrl)
    || (typeof process !== 'undefined' && process.env && process.env.BACKEND_API_URL)
    || 'http://localhost:8000';

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async (question, selectedText = null) => {
    if (!question.trim()) return;

    const userMessage = { id: messages.length + 1, text: question, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');

    try {
      const response = await fetch(`${BACKEND_API_URL}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: question, selected_text: selectedText }),
      });
      if (!response.ok) {
        const text = await response.text().catch(() => response.statusText || 'Unknown error');
        throw new Error(`Backend returned ${response.status}: ${text}`);
      }
      const data = await response.json();
      const botMessage = { 
        id: messages.length + 2, 
        text: data.answer, 
        detailed_text: data.detailed_answer, // Store detailed answer
        sender: 'bot', 
        source_references: data.source_references 
      };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errText = error && error.message ? `Error: ${error.message}` : 'Error: Could not connect to the chatbot. Please try again later.';
      const errorMessage = { id: messages.length + 2, text: errText, sender: 'bot' };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    }
  };

  const handleSendMessage = (e) => {
    e.preventDefault();
    sendMessage(input);
  };

  const handleAskAboutSelectedText = () => {
    if (currentSelectedText.trim()) {
      sendMessage("Explain this:", currentSelectedText);
      setCurrentSelectedText(''); // Clear selected text after sending
    }
  };

  const handleClearHistory = () => {
    setMessages([]);
    localStorage.removeItem(CHAT_HISTORY_KEY);
  };

  const BotMessage = ({ msg }) => {
    const [showFullAnswer, setShowFullAnswer] = useState(false);

    const toggleFullAnswer = () => setShowFullAnswer(!showFullAnswer);

    const handleFeedback = (feedback) => {
        console.log(`Feedback for message ${msg.id}: ${feedback}`);
        // Here you would typically send this feedback to a backend API
    };

    return (
      <div className={`${styles.message} ${styles[msg.sender]}`}>
        <p>{showFullAnswer && msg.detailed_text ? msg.detailed_text : msg.text}</p>
        {msg.detailed_text && msg.detailed_text.length > msg.text.length && (
          <button className={styles.showMoreButton} onClick={toggleFullAnswer}>
            {showFullAnswer ? 'Show Less' : 'Show More'}
          </button>
        )}
        {msg.source_references && msg.source_references.length > 0 && showFullAnswer && ( // Only show sources with full answer
          <div className={styles.sourceReferences}>
            <strong>Sources:</strong>
            <ul>
              {msg.source_references.map((src, index) => (
                <li key={index}>{src}</li>
              ))}
            </ul>
          </div>
        )}
        {msg.sender === 'bot' && (
            <div className={styles.feedbackButtons}>
                <button className={styles.feedbackButton} onClick={() => handleFeedback('upvote')}>👍</button>
                <button className={styles.feedbackButton} onClick={() => handleFeedback('downvote')}>👎</button>
            </div>
        )}
      </div>
    );
  };

  return (
    <>
      <button className={styles.floatingButton} onClick={toggleChat}>
        {isOpen ? 'X' : 'Chat'}
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
              <div style={{display:'flex',gap:12,alignItems:'center'}}>
                <div className={styles.headerAvatar}>AI</div>
                <div>
                  <h3>AI Chatbot</h3>
                  <div className="sub" className={styles.sub}>Ask about the textbook</div>
                </div>
              </div>
              <div>
                <button onClick={handleClearHistory} className={styles.clearHistoryButton}>Clear</button>
                <button onClick={toggleChat}>✕</button>
              </div>
            </div>
          <div className={styles.chatMessages}>
            {messages.map((msg) => (
              msg.sender === 'bot' ? (
                <BotMessage key={msg.id} msg={msg} />
              ) : (
                <div key={msg.id} className={`${styles.message} ${styles[msg.sender]}`}>
                  <p>{msg.text}</p>
                </div>
              )
            ))}
          </div>
          {currentSelectedText && (
            <div className={styles.selectedTextPrompt}>
              <p>Selected: "{currentSelectedText.substring(0, 50)}..."</p>
              <button onClick={handleAskAboutSelectedText}>Ask about this</button>
            </div>
          )}
          <form onSubmit={handleSendMessage} className={styles.chatInputForm}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question about the book..."
            />
            <button type="submit">Send</button>
          </form>
        </div>
      )}
    </>
  );
}

export default Chatbot;
