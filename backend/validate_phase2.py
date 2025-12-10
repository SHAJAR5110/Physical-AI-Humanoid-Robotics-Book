"""Manual validation script for Phase 2 - checks all imports and basic functionality."""

import sys
import traceback
from typing import Dict, List, Tuple

# Test results tracker
test_results: Dict[str, Tuple[bool, str]] = {}


def test_imports() -> bool:
    """Test all Phase 2 imports."""
    try:
        print("\n" + "="*60)
        print("TESTING IMPORTS")
        print("="*60)

        # Core services
        print("\n[OK] Testing embedding service import...")
        from src.services.embedding_service import EmbeddingService
        test_results["embedding_import"] = (True, "OK")

        print("[OK] Testing retrieval service import...")
        from src.services.retrieval_service import RetrievalService, RetrievalError
        test_results["retrieval_import"] = (True, "OK")

        print("[OK] Testing generation service import...")
        from src.services.generation_service import GenerationService, GenerationError
        test_results["generation_import"] = (True, "OK")

        print("[OK] Testing confidence service import...")
        from src.services.confidence_service import ConfidenceService
        test_results["confidence_import"] = (True, "OK")

        # Models
        print("[OK] Testing models import...")
        from src.models import (
            ChatRequest,
            ChatResponse,
            SourceRef,
            ErrorResponse,
            HealthResponse,
        )
        test_results["models_import"] = (True, "OK")

        # Utilities
        print("[OK] Testing error handler import...")
        from src.utils.error_handler import ErrorHandler
        test_results["error_handler_import"] = (True, "OK")

        print("[OK] Testing context manager import...")
        from src.utils.context_manager import ContextManager
        test_results["context_manager_import"] = (True, "OK")

        print("[OK] Testing validators import...")
        from src.utils.validators import InputValidator
        test_results["validators_import"] = (True, "OK")

        print("[OK] Testing response formatter import...")
        from src.utils.response_formatter import ResponseFormatter
        test_results["formatter_import"] = (True, "OK")

        print("[OK] Testing slug generator import...")
        from src.utils.slug_generator import SlugGenerator
        test_results["slug_import"] = (True, "OK")

        # Middleware
        print("[OK] Testing rate limit middleware import...")
        from src.middleware.rate_limit import RateLimitMiddleware
        test_results["rate_limit_import"] = (True, "OK")

        print("[OK] Testing logging middleware import...")
        from src.middleware.logging import LoggingMiddleware
        test_results["logging_import"] = (True, "OK")

        print("[OK] Testing config import...")
        from src.config import settings
        test_results["config_import"] = (True, "OK")

        return True

    except Exception as e:
        test_results["imports"] = (False, str(e))
        print(f"\n[FAIL] Import failed: {e}")
        traceback.print_exc()
        return False


def test_model_validation() -> bool:
    """Test Pydantic model validation."""
    try:
        print("\n" + "="*60)
        print("TESTING MODEL VALIDATION")
        print("="*60)

        from src.models import ChatRequest, ChatResponse, SourceRef

        # Test valid request
        print("\n[OK] Testing valid ChatRequest...")
        request = ChatRequest(question="What is ROS 2?")
        assert request.question == "What is ROS 2?"
        test_results["model_valid_request"] = (True, "OK")

        # Test invalid request (too short)
        print("[OK] Testing ChatRequest validation (too short)...")
        try:
            ChatRequest(question="ab")
            test_results["model_invalid_short"] = (False, "Should have failed")
            return False
        except Exception:
            test_results["model_invalid_short"] = (True, "Correctly rejected")

        # Test injection prevention
        print("[OK] Testing injection prevention...")
        try:
            ChatRequest(question="DROP TABLE users;")
            test_results["model_injection"] = (False, "Should have failed")
            return False
        except Exception:
            test_results["model_injection"] = (True, "Correctly blocked")

        # Test valid source
        print("[OK] Testing valid SourceRef...")
        source = SourceRef(
            chapter="Chapter 2",
            module="Module 2.1",
            section="ros-2",
            excerpt="Content here",
        )
        assert source.chapter == "Chapter 2"
        test_results["model_source"] = (True, "OK")

        # Test valid response
        print("[OK] Testing valid ChatResponse...")
        response = ChatResponse(
            answer="ROS 2 is a middleware for robotics communication.",
            sources=[source],
            confidence="high",
            processing_time_ms=1850,
        )
        assert response.confidence == "high"
        test_results["model_response"] = (True, "OK")

        return True

    except Exception as e:
        test_results["model_validation"] = (False, str(e))
        print(f"\n[FAIL] Model validation failed: {e}")
        traceback.print_exc()
        return False


def test_error_handling() -> bool:
    """Test error handling utilities."""
    try:
        print("\n" + "="*60)
        print("TESTING ERROR HANDLING")
        print("="*60)

        from src.utils.error_handler import ErrorHandler

        # Test rate limit error
        print("\n[OK] Testing rate limit error response...")
        response, status = ErrorHandler.handle_rate_limit()
        assert status == 429
        assert "Rate limit" in response.error
        test_results["error_rate_limit"] = (True, "OK")

        # Test service unavailable error
        print("[OK] Testing service unavailable error response...")
        response, status = ErrorHandler.handle_service_unavailable()
        assert status == 503
        assert "unavailable" in response.error.lower()
        test_results["error_unavailable"] = (True, "OK")

        # Test generic error
        print("[OK] Testing generic error response...")
        error = Exception("Test error")
        response, status = ErrorHandler.handle_generic_error(error)
        assert status == 500
        assert "error" in response.error.lower()
        test_results["error_generic"] = (True, "OK")

        return True

    except Exception as e:
        test_results["error_handling"] = (False, str(e))
        print(f"\n[FAIL] Error handling test failed: {e}")
        traceback.print_exc()
        return False


def test_utilities() -> bool:
    """Test utility functions."""
    try:
        print("\n" + "="*60)
        print("TESTING UTILITIES")
        print("="*60)

        from src.utils.validators import InputValidator
        from src.utils.context_manager import ContextManager
        from src.utils.slug_generator import SlugGenerator

        # Test input validators
        print("\n[OK] Testing InputValidator...")
        is_valid, msg = InputValidator.validate_question("What is ROS 2?")
        assert is_valid is True
        test_results["util_validator_valid"] = (True, "OK")

        # Test injection detection
        is_valid, msg = InputValidator.validate_question("DROP TABLE;")
        assert is_valid is False
        test_results["util_validator_injection"] = (True, "OK")

        # Test context manager
        print("[OK] Testing ContextManager...")
        context = ContextManager.prepare_context(
            [{"chapter": "Ch", "module": "Mod", "excerpt": "Content"}]
        )
        assert "Ch" in context
        test_results["util_context"] = (True, "OK")

        # Test slug generator
        print("[OK] Testing SlugGenerator...")
        slug = SlugGenerator.slugify("ROS 2 Topics")
        assert slug == "ros-2-topics"
        test_results["util_slug"] = (True, "OK")

        return True

    except Exception as e:
        test_results["utilities"] = (False, str(e))
        print(f"\n[FAIL] Utilities test failed: {e}")
        traceback.print_exc()
        return False


def test_config() -> bool:
    """Test configuration loading."""
    try:
        print("\n" + "="*60)
        print("TESTING CONFIGURATION")
        print("="*60)

        from src.config import settings

        print("\n[OK] Testing config loading...")
        assert settings.qdrant_url
        assert settings.groq_api_key
        test_results["config_loading"] = (True, "OK")

        print("[OK] Testing CORS origins parsing...")
        origins = settings.allowed_origins_list
        assert len(origins) > 0
        test_results["config_cors"] = (True, "OK")

        print(f"  - Loaded {len(origins)} allowed origins")
        for origin in origins:
            print(f"    - {origin}")

        return True

    except Exception as e:
        test_results["config"] = (False, str(e))
        print(f"\n[FAIL] Config test failed: {e}")
        traceback.print_exc()
        return False


def print_summary() -> None:
    """Print test summary."""
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)

    passed = sum(1 for result, _ in test_results.values() if result)
    total = len(test_results)

    print(f"\n[PASS] Passed: {passed}/{total}")

    if passed < total:
        print(f"[FAIL] Failed: {total - passed}")
        print("\nFailed tests:")
        for test_name, (result, msg) in test_results.items():
            if not result:
                print(f"  - {test_name}: {msg}")
    else:
        print("\n[SUCCESS] ALL TESTS PASSED!")

    return passed == total


def main() -> int:
    """Run all validation tests."""
    print("\n" + "="*60)
    print("PHASE 2 VALIDATION SCRIPT")
    print("Checking all imports, models, validation, and utilities")
    print("="*60)

    # Run all tests
    all_pass = True
    all_pass = test_imports() and all_pass
    all_pass = test_model_validation() and all_pass
    all_pass = test_error_handling() and all_pass
    all_pass = test_utilities() and all_pass
    all_pass = test_config() and all_pass

    # Print summary
    all_pass = print_summary() and all_pass

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
