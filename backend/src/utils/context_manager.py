"""Context window management for token limits."""

import logging

logger = logging.getLogger(__name__)


class ContextManager:
    """Manages context window size for LLM to stay within token limits."""

    # Rough token estimation (avg ~4 chars per token)
    CHARS_PER_TOKEN = 4
    MAX_CONTEXT_TOKENS = 6000
    MAX_CONTEXT_CHARS = MAX_CONTEXT_TOKENS * CHARS_PER_TOKEN

    @staticmethod
    def prepare_context(
        sources: list[dict],
        max_tokens: int = MAX_CONTEXT_TOKENS,
    ) -> str:
        """
        Prepare formatted context from sources, respecting token limits.

        Args:
            sources: List of source references with excerpts
            max_tokens: Maximum tokens allowed

        Returns:
            Formatted context string
        """
        max_chars = max_tokens * ContextManager.CHARS_PER_TOKEN
        context_parts = []
        current_length = 0

        for source in sources:
            excerpt = source.get("excerpt", "")
            if not excerpt:
                continue

            # Format source with metadata
            formatted = (
                f"From {source.get('chapter', 'Unknown Chapter')}, "
                f"{source.get('module', 'Unknown Module')}:\n"
                f"{excerpt}\n"
                f"---\n"
            )

            # Check if adding this source would exceed limit
            if current_length + len(formatted) > max_chars:
                # Try to fit partial excerpt
                remaining = max_chars - current_length
                if remaining > 100:  # Only add if meaningful amount remains
                    truncated = formatted[:remaining] + "\n[truncated]"
                    context_parts.append(truncated)
                break

            context_parts.append(formatted)
            current_length += len(formatted)

        context = "\n".join(context_parts)
        token_estimate = len(context) / ContextManager.CHARS_PER_TOKEN

        logger.debug(
            f"Prepared context: {len(context)} chars (~{token_estimate:.0f} tokens)"
        )

        return context

    @staticmethod
    def estimate_tokens(text: str) -> int:
        """
        Rough estimate of token count.

        Args:
            text: Text to estimate

        Returns:
            Approximate token count
        """
        return max(1, int(len(text) / ContextManager.CHARS_PER_TOKEN))

    @staticmethod
    def validate_token_count(text: str, max_tokens: int) -> bool:
        """
        Validate if text fits within token limit.

        Args:
            text: Text to validate
            max_tokens: Maximum allowed tokens

        Returns:
            True if within limit, False otherwise
        """
        estimated = ContextManager.estimate_tokens(text)
        return estimated <= max_tokens
