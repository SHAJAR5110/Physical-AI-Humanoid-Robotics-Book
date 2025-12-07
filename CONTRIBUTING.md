# Contributing to Physical AI & Humanoid Robotics

Thank you for your interest in contributing to this open-source educational project! This document provides guidelines for contributing to our book, code, and documentation.

## How to Contribute

### 1. Report Issues

Found a bug or have a suggestion?
- Check existing [GitHub Issues](https://github.com/SHAJAR5110/Physical-AI-Humanoid-Robotics-Book/issues)
- Create a new issue with a clear description and reproduction steps
- Use labels: `bug`, `enhancement`, `documentation`, `question`

### 2. Submit Content

#### Adding or Updating Chapters

**Location**: `book-source/docs/`

**Guidelines**:
- Follow the markdown format of existing chapters
- Aim for 2,500-3,500 words per chapter
- Include practical code examples (minimum 3-5 per chapter)
- Structure with clear headings and sections
- Add explanatory comments in code snippets
- Include visual diagrams where helpful (PNG/SVG in `book-source/static/`)

**Template Structure**:
```markdown
# Chapter Title

## Introduction
Brief overview of what readers will learn.

## Key Concepts
### Concept 1
Explanation and context.

### Concept 2
Explanation and context.

## Code Examples
### Example 1: Title
```python
# Code here
```
Explanation of what this example demonstrates.

## Best Practices
- Practice 1: Explanation
- Practice 2: Explanation

## Conclusion
Summary and next steps.

## Further Reading
- [Resource 1](link)
- [Resource 2](link)
```

#### Adding Code Examples

**Location**: Code examples embedded in chapters or `book-source/static/examples/`

**Guidelines**:
- Always include language identifier in code blocks
- Add explanatory comments
- Show realistic input/output
- Keep examples focused on one concept
- Test examples before submitting
- Link to full source repositories where applicable

**Example Format**:
```markdown
### Example: ROS2 Service Call
\`\`\`python
#!/usr/bin/env python3
# Example demonstrating ROS2 service call

from ros2_example import MyService

# Create service client
client = MyService()

# Call service with parameters
response = client.add_two_ints(a=5, b=3)
print(f"Result: {response.sum}")  # Output: 8
\`\`\`

This example shows how to create and call a ROS2 service...
\`\`\`
```

### 3. Contribute Code

#### Frontend (React/TypeScript)

**Location**: `book-source/src/`

**Setup**:
```bash
cd book-source
npm install
npm start
```

**Guidelines**:
- Use TypeScript for all new components
- Follow existing CSS module patterns
- Implement dark mode support
- Ensure mobile responsiveness
- Add proper error handling
- Include comments for complex logic

**Example Component**:
```typescript
import React, { useState } from 'react';
import styles from './MyComponent.module.css';

interface MyComponentProps {
  title: string;
  onAction: () => void;
}

export const MyComponent: React.FC<MyComponentProps> = ({
  title,
  onAction
}) => {
  const [loading, setLoading] = useState(false);

  return (
    <div className={styles.container}>
      <h2>{title}</h2>
      {/* Component JSX */}
    </div>
  );
};
```

#### Backend (Python/FastAPI)

**Location**: `backend/src/`

**Setup**:
```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

**Guidelines**:
- Use type hints for all functions
- Follow PEP 8 style guide
- Add docstrings to functions/classes
- Include error handling
- Write tests for new functionality
- Use Pydantic for validation

**Example Endpoint**:
```python
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

router = APIRouter()

class RequestSchema(BaseModel):
    """Request validation schema"""
    name: str
    email: str

@router.post("/endpoint")
async def my_endpoint(request: RequestSchema) -> dict:
    """
    Endpoint description.

    Args:
        request: Request data

    Returns:
        Response data dictionary

    Raises:
        HTTPException: If validation fails
    """
    # Implementation here
    return {"status": "success"}
```

### 4. Documentation

**Locations**:
- Technical docs: Markdown files in `book-source/docs/`
- API docs: Docstrings in Python code
- README files in each directory
- Architecture: `specs/001-mvp-features/plan.md`

**Guidelines**:
- Use clear, concise language
- Include examples
- Add links to related resources
- Keep docs updated with code changes
- Use proper markdown formatting

## Development Workflow

### 1. Fork and Clone
```bash
git clone https://github.com/YOUR_USERNAME/Physical-AI-Humanoid-Robotics-Book.git
cd Physical-AI-Humanoid-Robotics-Book
git checkout 001-mvp-features
```

### 2. Create Feature Branch
```bash
git checkout -b feature/your-feature-name
```

**Branch Naming**:
- Features: `feature/description`
- Fixes: `fix/description`
- Docs: `docs/description`
- Chapters: `chapter/chapter-name`

### 3. Make Changes
- Keep commits focused and atomic
- Write clear commit messages
- Reference issues: `Fixes #123`
- Test thoroughly before committing

### 4. Commit Message Format
```
type: short description (50 chars max)

Detailed description of what changed and why (72 chars per line).
Reference any related issues.

Fixes #123
```

**Types**:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `style`: Code style (formatting, etc.)
- `refactor`: Code refactoring
- `test`: Adding tests
- `chore`: Maintenance tasks

### 5. Push and Create Pull Request
```bash
git push origin feature/your-feature-name
```

Then create a PR on GitHub with:
- Clear description of changes
- Reference to related issues
- Screenshots for UI changes
- Test results

## PR Review Process

### What We Look For
- ✅ Code quality and style consistency
- ✅ Proper error handling
- ✅ Tests for new functionality
- ✅ Documentation updates
- ✅ No breaking changes (or justified)
- ✅ Clear commit history

### Timeline
- Initial review: 2-3 business days
- Changes requested: 1-2 business days to respond
- Approved PRs: Merged promptly

## Code Standards

### JavaScript/TypeScript
```typescript
// Use strict typing
const myFunction = (param: string): void => {
  // Implementation
};

// Use meaningful names
const isAuthenticated = true;
const getUserProfile = async () => {};

// Add comments for complex logic
// Retry mechanism with exponential backoff
```

### Python
```python
"""Module docstring."""

def my_function(param: str) -> dict:
    """
    Function docstring.

    Args:
        param: Parameter description

    Returns:
        Dictionary with results
    """
    # Implementation
    return {}
```

### Markdown
```markdown
# Heading 1
## Heading 2
### Heading 3

- Bullet points
- Use clear language

1. Numbered lists
2. For procedures

**Bold** for emphasis
*Italic* for emphasis

[Link text](url)
```

## Testing

### Frontend
```bash
cd book-source
npm run build  # Verify build succeeds
```

### Backend
```bash
cd backend
python -m pytest
```

### Manual Testing
- Test on multiple browsers (Chrome, Firefox, Safari)
- Test on mobile devices
- Test dark mode
- Test error scenarios

## Style Guide

### Writing Style
- Use active voice
- Be concise and clear
- Avoid jargon without explanation
- Use examples liberally
- Include practical applications

### Code Style
- Keep functions small and focused
- Use meaningful variable names
- Add comments for "why" not "what"
- Follow language conventions
- Maintain consistency with existing code

## Licensing

By contributing to this project, you agree that your contributions will be licensed under the **MIT License**.

## Community Guidelines

- Be respectful and inclusive
- Assume good intent
- Focus on the code, not the person
- Help others learn
- Celebrate contributions

## Getting Help

- **Questions**: Open a GitHub Discussion
- **Issues**: Report bugs with clear reproduction steps
- **Community**: Check existing issues/PRs before asking
- **Documentation**: Review `book-source/README.md`

## Recognition

Contributors will be recognized in:
- GitHub contributors page
- CONTRIBUTORS.md (coming soon)
- Commit history

## Code of Conduct

This project follows the [Contributor Covenant Code of Conduct](https://www.contributor-covenant.org/). By participating, you are expected to uphold this code.

## Questions?

- 📖 Read existing documentation
- 🔍 Search closed issues
- 💬 Open a discussion
- 📧 Contact maintainers

---

**Thank you for contributing!** Your work helps make Physical AI education accessible to everyone.

**Happy coding!** 🚀
