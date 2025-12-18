import os
import json
import logging
from typing import Dict, List, Any
from dotenv import load_dotenv
from groq import Groq
import time

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGAgent:
    def __init__(self):
        # Initialize Groq client for LLM
        self.groq_client = Groq(api_key=os.getenv("GROQ_API_KEY"))

        # Initialize retriever for RAG
        from retrieving import RAGRetriever
        self.retriever = RAGRetriever()

        logger.info("RAG Agent initialized with Groq LLM")

    def query_agent(self, query_text: str) -> Dict:
        """
        Process a query through the RAG agent and return structured response
        """
        start_time = time.time()

        logger.info(f"Processing query through RAG agent: '{query_text[:50]}...'")

        try:
            # Step 1: Retrieve relevant documents from Qdrant (get more candidates)
            retrieval_response = self.retriever.retrieve(
                query_text=query_text,
                top_k=15,  # Increased from 5 to get more candidates
                threshold=0.0  # Will filter manually below
            )

            results = json.loads(retrieval_response)
            all_chunks = results.get("results", [])

            # Step 1b: Filter to high-quality matches (similarity >= 0.3)
            # Lowered threshold to ensure we get usable chunks
            matched_chunks = [
                chunk for chunk in all_chunks
                if chunk.get("similarity_score", 0) >= 0.3
            ]

            # Log detailed retrieval stats with similarity scores
            logger.info(f"Retrieved {len(all_chunks)} total chunks, {len(matched_chunks)} passed quality filter (>= 0.3 similarity)")
            if all_chunks:
                logger.info(f"Top 3 similarity scores: {[round(c.get('similarity_score', 0), 3) for c in all_chunks[:3]]}")
                logger.info(f"Query: '{query_text}'")
                # Log first chunk content preview for debugging
                if all_chunks[0].get("content"):
                    preview = all_chunks[0].get("content", "")[:100]
                    logger.info(f"Top match preview: {preview}...")

            # Step 2: Build context from filtered high-quality chunks
            context, has_good_context = self._build_context(matched_chunks)

            # Step 3: Generate response using Groq
            answer = self._generate_answer(query_text, context, has_good_context)

            # Step 4: Extract sources only from chunks that were used
            sources = [chunk.get("url", "") for chunk in matched_chunks if chunk.get("url")]

            # Calculate query time
            query_time_ms = (time.time() - start_time) * 1000

            # Step 5: Calculate confidence based on filtered retrieval results
            confidence = self._calculate_confidence(matched_chunks, has_good_context)

            # Format the response
            response = {
                "answer": answer,
                "sources": sources,
                "matched_chunks": matched_chunks,
                "query_time_ms": query_time_ms,
                "confidence": confidence
            }

            logger.info(f"Query processed in {query_time_ms:.2f}ms with confidence: {confidence}")
            return response

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            return {
                "answer": "Sorry, I encountered an error processing your request.",
                "sources": [],
                "matched_chunks": [],
                "error": str(e),
                "query_time_ms": (time.time() - start_time) * 1000,
                "confidence": "low"
            }

    def _build_context(self, matched_chunks: List[Dict]) -> tuple:
        """
        Build context string from matched chunks.
        Returns: (context_string, has_good_context_flag)
        """
        if not matched_chunks:
            return "No relevant information found in the textbook for this query.", False

        # Limit to top 10 chunks to avoid context overflow
        top_chunks = matched_chunks[:10]

        context_parts = []
        high_quality_count = 0

        for i, chunk in enumerate(top_chunks, 1):
            content = chunk.get("content", "").strip()
            url = chunk.get("url", "")
            similarity = chunk.get("similarity_score", 0)

            # Track high-quality chunks (similarity >= 0.6)
            if similarity >= 0.6:
                high_quality_count += 1

            if content:  # Only add non-empty chunks
                # Truncate very long content to avoid overwhelming the LLM
                if len(content) > 500:
                    content = content[:500] + "..."

                context_parts.append(
                    f"[Source {i}] (Relevance: {similarity:.1%})\n{content}"
                )

        if not context_parts:
            return "No relevant information found in the textbook for this query.", False

        # Mark as "good context" if we have at least one decent match (>= 0.5 similarity)
        # OR if we have multiple matches (>= 0.3)
        has_good_context = high_quality_count >= 1 or len(context_parts) >= 2

        context = "\n\n---\n\n".join(context_parts)

        logger.info(f"Context quality: high_quality_count={high_quality_count}, has_good_context={has_good_context}, context_parts={len(context_parts)}")

        return context, has_good_context

    def _generate_answer(self, query: str, context: str, has_good_context: bool) -> str:
        """
        Generate an answer using Groq LLM based on retrieved context.
        If context quality is poor, refuse to answer with generic knowledge.
        """
        # System prompt: Prefer textbook content but use what we have
        system_prompt = """You are an AI assistant for the Physical AI and Humanoid Robotics textbook.

INSTRUCTIONS:
1. Base your answer primarily on the provided textbook context
2. Use the provided Sources when answering
3. If the context is clearly relevant, provide a helpful answer based on it
4. If the context provided does NOT help answer the question, say:
   "I don't have information about this topic in the provided textbook sections. Please try a different question."
5. Do not introduce information from outside the provided context
6. When citing information, reference the Source number

Your primary role is to faithfully represent the textbook content."""

        # If we don't have good context, note this to the LLM
        context_quality_note = ""
        if not has_good_context:
            context_quality_note = "\n\nNOTE: The provided context may have limited relevance. Use it only if it actually helps answer the question."
        else:
            context_quality_note = "\n\nNOTE: The provided context appears relevant to the question."

        user_message = f"""Textbook Content (from Physical AI & Humanoid Robotics):

{context}

{context_quality_note}

---

User Question: {query}

Answer ONLY based on the textbook content above. Do not provide generic knowledge."""

        try:
            completion = self.groq_client.chat.completions.create(
                model="openai/gpt-oss-120b",  # Groq model
                max_tokens=1024,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.3  # Lower temperature for more factual, less creative responses
            )

            answer = completion.choices[0].message.content
            logger.info(f"Generated answer: {answer[:100]}...")
            return answer

        except Exception as e:
            logger.error(f"Error generating answer with Groq: {e}")
            return f"I encountered an error generating a response: {str(e)}"

    def _calculate_confidence(self, matched_chunks: List[Dict], has_good_context: bool) -> str:
        """
        Calculate confidence level based on similarity scores and number of matches.
        """
        if not matched_chunks:
            return "low"

        # Get top chunk's similarity (most relevant match)
        best_score = matched_chunks[0].get("similarity_score", 0.0)
        num_chunks = len(matched_chunks)

        # Consider average of top chunks
        top_5 = matched_chunks[:5]
        avg_score = sum(c.get("similarity_score", 0.0) for c in top_5) / len(top_5)

        # High confidence: best >= 0.7 AND (has_good_context OR multiple matches)
        if best_score >= 0.7 and (has_good_context or num_chunks >= 3):
            return "high"
        # Medium confidence: best >= 0.5 OR (avg >= 0.4 AND multiple matches)
        elif best_score >= 0.5 or (avg_score >= 0.4 and num_chunks >= 2):
            return "medium"
        # Low confidence: weak matches or limited context
        else:
            return "low"

def query_agent(query_text: str) -> Dict:
    """
    Convenience function to query the RAG agent
    """
    agent = RAGAgent()
    return agent.query_agent(query_text)

def run_agent_sync(query_text: str) -> Dict:
    """
    Synchronous function to run the agent for direct usage
    """
    agent = RAGAgent()
    return agent.query_agent(query_text)

def main():
    """
    Main function to demonstrate the RAG agent functionality
    """
    logger.info("Initializing RAG Agent...")

    # Initialize the agent
    agent = RAGAgent()

    # Example queries to test the system
    test_queries = [
        "What is ROS2?",
        "Explain humanoid design principles",
        "How does VLA work?",
        "What are simulation techniques?",
        "Explain AI control systems"
    ]

    print("RAG Agent - Testing Queries")
    print("=" * 50)

    for i, query in enumerate(test_queries, 1):
        print(f"\nQuery {i}: {query}")
        print("-" * 30)

        # Process query through agent
        response = agent.query_agent(query)

        # Print formatted results
        print(f"Answer: {response['answer']}")

        if response.get('sources'):
            print(f"Sources: {len(response['sources'])} documents")
            for source in response['sources'][:3]:  # Show first 3 sources
                print(f"  - {source}")

        if response.get('matched_chunks'):
            print(f"Matched chunks: {len(response['matched_chunks'])}")
            for j, chunk in enumerate(response['matched_chunks'][:2], 1):  # Show first 2 chunks
                content_preview = chunk['content'][:100] + "..." if len(chunk['content']) > 100 else chunk['content']
                print(f"  Chunk {j}: {content_preview}")
                print(f"    Source: {chunk['url']}")
                print(f"    Score: {chunk['similarity_score']:.3f}")

        print(f"Query time: {response['query_time_ms']:.2f}ms")
        print(f"Confidence: {response.get('confidence', 'unknown')}")

        if i < len(test_queries):  # Don't sleep after the last query
            time.sleep(1)  # Small delay between queries

if __name__ == "__main__":
    main()
