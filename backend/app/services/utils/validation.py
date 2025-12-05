"""
Request validation utilities for chat endpoint with selected text support.

Provides validation functions for selected_text, question, and other request parameters.
"""

from typing import Optional
import re


def validate_selected_text(text: Optional[str], max_length: int = 500) -> bool:
    """
    Validate selected text parameter.

    Checks:
    - Not None or empty (optional field, but if provided must be valid)
    - Length 0-500 characters
    - Valid UTF-8 encoding
    - Not only whitespace

    Args:
        text: Selected text to validate
        max_length: Maximum allowed length (default 500)

    Returns:
        True if valid, False otherwise
    """
    # None or empty is valid (optional field)
    if text is None or text == '':
        return True

    # Must be string
    if not isinstance(text, str):
        return False

    # Check length
    if len(text) > max_length:
        return False

    # Check if only whitespace
    if not text.strip():
        return False

    # Check if valid UTF-8 (should be automatic in Python 3, but explicit check)
    try:
        text.encode('utf-8')
    except (UnicodeEncodeError, AttributeError):
        return False

    return True


def validate_question(question: str, min_length: int = 1, max_length: int = 2000) -> bool:
    """
    Validate user question parameter.

    Checks:
    - Not None or empty
    - Length 1-2000 characters
    - Not only whitespace

    Args:
        question: Question to validate
        min_length: Minimum allowed length (default 1)
        max_length: Maximum allowed length (default 2000)

    Returns:
        True if valid, False otherwise
    """
    # Must not be None
    if question is None:
        return False

    # Must be string
    if not isinstance(question, str):
        return False

    # Check if only whitespace
    if not question.strip():
        return False

    # Check length
    text_length = len(question)
    if text_length < min_length or text_length > max_length:
        return False

    return True


def normalize_selected_text(text: Optional[str]) -> Optional[str]:
    """
    Normalize selected text by trimming whitespace.

    Converts multiple spaces to single space, strips leading/trailing whitespace.
    Returns None if text becomes empty after normalization.

    Args:
        text: Text to normalize

    Returns:
        Normalized text or None if empty

    Example:
        >>> normalize_selected_text("  forward   kinematics  ")
        'forward kinematics'

        >>> normalize_selected_text("   ")
        None
    """
    if text is None:
        return None

    if not isinstance(text, str):
        return None

    # Strip leading/trailing whitespace
    normalized = text.strip()

    # Collapse multiple spaces into single space
    normalized = re.sub(r'\s+', ' ', normalized)

    # Return None if empty after normalization
    if not normalized:
        return None

    return normalized


def normalize_question(question: str) -> str:
    """
    Normalize question by trimming whitespace.

    Args:
        question: Question to normalize

    Returns:
        Normalized question

    Example:
        >>> normalize_question("  What is   forward kinematics?  ")
        'What is forward kinematics?'
    """
    if not isinstance(question, str):
        return question

    # Strip leading/trailing whitespace
    normalized = question.strip()

    # Collapse multiple spaces into single space
    normalized = re.sub(r'\s+', ' ', normalized)

    return normalized


def validate_request_parameters(
    question: str,
    selected_text: Optional[str] = None
) -> tuple[bool, Optional[str]]:
    """
    Validate all request parameters together.

    Args:
        question: User question
        selected_text: Optional selected text

    Returns:
        Tuple of (is_valid, error_message)
        - If valid: (True, None)
        - If invalid: (False, "error description")
    """
    # Validate question (required)
    if not validate_question(question):
        question_length = len(question) if question else 0
        if question_length == 0:
            return False, "Question cannot be empty"
        elif question_length < 1:
            return False, "Question must be at least 1 character"
        elif question_length > 2000:
            return False, "Question must not exceed 2000 characters"
        else:
            return False, "Invalid question format"

    # Validate selected_text (optional)
    if not validate_selected_text(selected_text):
        if selected_text:
            selected_text_length = len(selected_text) if selected_text else 0
            if selected_text_length > 500:
                return False, "Selected text must not exceed 500 characters"
            else:
                return False, "Invalid selected text format"

    return True, None


def sanitize_parameters(
    question: str,
    selected_text: Optional[str] = None
) -> dict:
    """
    Sanitize request parameters.

    Applies normalization and returns clean parameters.

    Args:
        question: User question
        selected_text: Optional selected text

    Returns:
        Dictionary with sanitized parameters
    """
    return {
        'question': normalize_question(question),
        'selected_text': normalize_selected_text(selected_text),
    }


def get_validation_error_message(field: str, error_code: str) -> str:
    """
    Get user-friendly error message for validation failures.

    Args:
        field: Field name that failed validation
        error_code: Error code (e.g., 'too_long', 'too_short', 'invalid_format')

    Returns:
        User-friendly error message
    """
    messages = {
        'question': {
            'required': 'Question is required',
            'too_short': 'Question must be at least 1 character',
            'too_long': 'Question must not exceed 2000 characters',
            'empty': 'Question cannot be empty or only whitespace',
            'invalid_format': 'Question format is invalid',
        },
        'selected_text': {
            'too_long': 'Selected text must not exceed 500 characters',
            'empty': 'Selected text cannot be empty when provided',
            'invalid_format': 'Selected text format is invalid',
        },
    }

    field_messages = messages.get(field, {})
    return field_messages.get(error_code, f'Validation error in {field}')
