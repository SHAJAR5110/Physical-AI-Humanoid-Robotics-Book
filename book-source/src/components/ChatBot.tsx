/**
 * ChatBot Component - Floating widget for Q&A about the Physical AI textbook.
 *
 * Features:
 * - Toggle button (💬) fixed in bottom-left corner
 * - Collapsible floating card widget
 * - Question form with optional text selection
 * - Real-time API integration with backend
 * - Answer display with sources and confidence
 * - Feedback buttons (thumbs up/down)
 * - Mobile responsive design
 * - Loading states and error handling
 *
 * Environment Variables (in .env or docusaurus.config.js):
 * - VITE_API_BASE_URL: Backend API URL (e.g., http://localhost:8000 or https://api.example.com)
 *   If not set, defaults to http://localhost:8000
 */

import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatBot.module.css';

// Types for API responses
interface Source {
  chapter: string;
  module: string;
  section: string;
  excerpt: string;
  similarity?: number;
}

interface ChatResponse {
  answer: string;
  sources: Source[];
  confidence: 'high' | 'medium' | 'low';
  processing_time_ms: number;
}

interface ChatRequest {
  question: string;
  selected_text?: string;
}

/**
 * ChatBot Component - Main floating widget
 */
const ChatBot: React.FC = () => {
  // State management
  const [isOpen, setIsOpen] = useState(false);
  const [question, setQuestion] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [response, setResponse] = useState<ChatResponse | null>(null);
  const [showTimeoutWarning, setShowTimeoutWarning] = useState(false);
  const textareaRef = useRef<HTMLTextAreaElement>(null);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);

  // API base URL - connects to backend
  // For local development: http://localhost:8000
  // For production: update this URL to your deployed backend
  const API_BASE_URL = 'http://localhost:8000';

  // Handle text selection in page
  useEffect(() => {
    const handleTextSelection = () => {
      const selected = window.getSelection()?.toString() || '';
      if (selected.length > 0 && selected.length < 500) {
        setSelectedText(selected);
        setIsOpen(true); // Auto-open widget when text is selected
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => document.removeEventListener('mouseup', handleTextSelection);
  }, []);

  // Handle form submission
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!question.trim()) {
      setError('Please enter a question');
      return;
    }

    setIsLoading(true);
    setError(null);
    setShowTimeoutWarning(false);

    // Set timeout warning after 2 seconds
    timeoutRef.current = setTimeout(() => {
      setShowTimeoutWarning(true);
    }, 2000);

    try {
      const payload = {
        query: question.trim(),
        chat_history: [], // Backend expects chat_history array
      };

      const res = await fetch(`${API_BASE_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (!res.ok) {
        const errorData = await res.json();
        throw new Error(errorData.details || 'Failed to get response');
      }

      const data = await res.json();

      // Map backend response to frontend format
      // Handle both /ask and /api/chat response formats
      const sources = (data.sources || []).map((source: any) => ({
        chapter: source.chapter || 'Unknown',
        module: source.module || 'Content',
        section: source.section || 'content',
        excerpt: source.content || source.excerpt || '',
        similarity: source.similarity_score || source.similarity,
      }));

      const formattedResponse: ChatResponse = {
        answer: data.response || data.answer || '',
        sources: sources,
        confidence: (data.confidence || 'medium') as 'high' | 'medium' | 'low',
        processing_time_ms: data.processing_time_ms || data.query_time_ms || 0,
      };
      setResponse(formattedResponse);
      setQuestion(''); // Clear input after successful submission
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred. Please try again.');
    } finally {
      setIsLoading(false);
      setShowTimeoutWarning(false);
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
    }
  };

  // Handle feedback buttons
  const handleFeedback = (type: 'up' | 'down') => {
    // Log feedback for future integration
    console.log(`Feedback: ${type}`, { question, response });
    // TODO: Send feedback to analytics/database in future
  };

  // Handle refresh/clear conversation
  const handleRefresh = () => {
    setResponse(null);
    setQuestion('');
    setSelectedText('');
    setError(null);
    if (textareaRef.current) {
      textareaRef.current.focus();
    }
  };

  // Get confidence indicator
  const getConfidenceIcon = (confidence: string) => {
    switch (confidence) {
      case 'high':
        return '✅';
      case 'medium':
        return '⚠️';
      case 'low':
        return '❓';
      default:
        return '❓';
    }
  };

  // Close widget
  const closeWidget = () => {
    setIsOpen(false);
  };

  return (
    <>
      {/* Toggle Button */}
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat widget"
        title="Ask about the book"
      >
        💬
      </button>

      {/* Widget Card */}
      {isOpen && (
        <div className={styles.widget}>
          {/* Header */}
          <div className={styles.header}>
            <h3 className={styles.title}>Ask about the book</h3>
            <button
              className={styles.closeButton}
              onClick={closeWidget}
              aria-label="Close chat widget"
            >
              ✕
            </button>
          </div>

          {/* Content */}
          <div className={styles.content}>
            {/* Input Form */}
            {!response && (
              <form onSubmit={handleSubmit} className={styles.form}>
                {selectedText && (
                  <div className={styles.selectedTextNotice}>
                    <small>Selected: "{selectedText.substring(0, 50)}..."</small>
                    <button
                      type="button"
                      className={styles.clearSelected}
                      onClick={() => setSelectedText('')}
                    >
                      Clear
                    </button>
                  </div>
                )}

                <textarea
                  ref={textareaRef}
                  value={question}
                  onChange={(e) => setQuestion(e.target.value)}
                  placeholder="Ask a question about the book..."
                  className={styles.input}
                  rows={3}
                  disabled={isLoading}
                />

                <button
                  type="submit"
                  className={styles.submitButton}
                  disabled={isLoading || !question.trim()}
                >
                  {isLoading ? '⏳ Loading...' : 'Send'}
                </button>

                {showTimeoutWarning && (
                  <p className={styles.timeoutWarning}>
                    Still loading... The backend might be warming up. Please wait.
                  </p>
                )}
              </form>
            )}

            {/* Error State */}
            {error && !response && (
              <div className={styles.error}>
                <p>{error}</p>
                <button
                  className={styles.dismissError}
                  onClick={() => setError(null)}
                >
                  Dismiss
                </button>
              </div>
            )}

            {/* Loading State */}
            {isLoading && !response && (
              <div className={styles.loading}>
                <div className={styles.spinner}></div>
                <p>Searching the book...</p>
              </div>
            )}

            {/* Response Display */}
            {response && (
              <div className={styles.response}>
                {/* Answer */}
                <div className={styles.answer}>
                  <p>{response.answer}</p>
                </div>

                {/* Confidence Indicator */}
                <div className={styles.confidence}>
                  <span className={styles.confidenceIcon}>
                    {getConfidenceIcon(response.confidence)}
                  </span>
                  <span className={styles.confidenceLabel}>
                    {response.confidence} confidence
                  </span>
                  <span className={styles.processingTime}>
                    ({response.processing_time_ms}ms)
                  </span>
                </div>

                {/* Sources */}
                {response.sources && response.sources.length > 0 && (
                  <div className={styles.sources}>
                    <h4>Sources:</h4>
                    <ul>
                      {response.sources.map((source, idx) => (
                        <li key={idx} className={styles.sourceItem}>
                          <a
                            href={getChapterDocUrl(source.chapter, source.section)}
                            target="_blank"
                            rel="noopener noreferrer"
                            className={styles.sourceLink}
                          >
                            {source.module}
                          </a>
                          {source.similarity && (
                            <span className={styles.similarity}>
                              {(source.similarity * 100).toFixed(0)}%
                            </span>
                          )}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}

                {/* Feedback Buttons */}
                <div className={styles.feedback}>
                  <button
                    className={styles.feedbackButton}
                    onClick={() => handleFeedback('up')}
                    title="This answer was helpful"
                  >
                    👍
                  </button>
                  <button
                    className={styles.feedbackButton}
                    onClick={() => handleFeedback('down')}
                    title="This answer wasn't helpful"
                  >
                    👎
                  </button>
                  <button
                    className={styles.refreshButton}
                    onClick={handleRefresh}
                    title="Ask another question"
                  >
                    🔄
                  </button>
                </div>
              </div>
            )}
          </div>
        </div>
      )}
    </>
  );
};

/**
 * Helper function to get Docusaurus URL from chapter name and section
 * Maps chapter names to the actual markdown file structure
 */
function getChapterDocUrl(chapter: string, section: string): string {
  // Map chapter names to doc file names
  const chapterMap: { [key: string]: string } = {
    'Chapter 1': 'intro',
    'Chapter 2': 'ros2',
    'Chapter 3': 'gazebo',
    'Chapter 4': 'isaac',
    'Chapter 5': 'vla',
    'Chapter 6': 'capstone',
  };

  // Extract chapter number from chapter name (e.g., "Chapter 2: ROS 2 Fundamentals" -> "Chapter 2")
  const chapterKey = Object.keys(chapterMap).find(key => chapter.startsWith(key));
  const docFileName = chapterKey ? chapterMap[chapterKey] : 'intro';

  // Construct the URL: /docs/{filename}#{section}
  return `/docs/${docFileName}#${section}`;
}

export default ChatBot;
