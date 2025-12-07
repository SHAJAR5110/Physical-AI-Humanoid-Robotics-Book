# Physical AI & Humanoid Robotics - Book Source

An open-source educational resource for learning Physical AI, robotics frameworks, and humanoid systems. Built with [Docusaurus](https://docusaurus.io/).

## Overview

This repository contains the complete source code for the **Physical AI & Humanoid Robotics** interactive book, featuring:

- **6 Comprehensive Chapters** (17,500+ words)
  - Chapter 1: Physical AI Fundamentals
  - Chapter 2: ROS 2 Fundamentals
  - Chapter 3: Gazebo Simulation
  - Chapter 4: NVIDIA Isaac Platform
  - Chapter 5: Vision-Language-Action Models
  - Chapter 6: Capstone Project

- **45+ Code Examples** in Python, YAML, and JSON
- **Interactive Components** with dark mode support
- **Mobile-Responsive Design** for all devices
- **Complete Authentication System** (frontend ready)

## Quick Start

### Prerequisites
```
Node.js 18+
npm or yarn
```

### Installation

```bash
npm install
# or
yarn install
```

### Local Development

```bash
npm start
# or
yarn start
```

Starts a local development server at `http://localhost:3000` with hot reload.

### Production Build

```bash
npm run build
# or
yarn build
```

Generates optimized static content in the `build/` directory.

## Project Structure

```
book-source/
├── docs/                          # Chapter content (Markdown)
│   ├── 01_intro.md               # Physical AI Fundamentals
│   ├── 02_ros2.md                # ROS 2 Fundamentals
│   ├── 03_gazebo.md              # Gazebo Simulation
│   ├── 04_isaac.md               # NVIDIA Isaac Platform
│   ├── 05_vla.md                 # Vision-Language-Action Models
│   └── 06_capstone.md            # Capstone Project
├── src/
│   ├── pages/                    # Custom pages (Auth, etc.)
│   ├── components/               # React components
│   │   ├── AuthForm.tsx
│   │   ├── SignUpForm.tsx
│   │   └── SignInForm.tsx
│   ├── hooks/                    # Custom React hooks
│   │   └── useAuth.ts            # Authentication logic
│   └── css/                      # Styles and theming
├── static/                        # Images, assets
├── docusaurus.config.ts           # Site configuration
├── sidebars.ts                    # Navigation structure
├── tailwind.config.js             # Tailwind CSS config
└── package.json                   # Dependencies
```

## Features

### Educational Content
- **Step-by-step tutorials** for robotics concepts
- **Real-world examples** from production systems
- **Practical code samples** you can run immediately
- **Visual diagrams** and architecture explanations

### Developer Experience
- **TypeScript** for type safety
- **React Hooks** for state management
- **CSS Modules** with dark mode
- **Responsive Design** from mobile to desktop
- **Fast reload** during development

### Authentication Ready
- User signup and signin forms
- Profile customization (OS, GPU, experience level)
- Persistent session management
- Integration with FastAPI backend

## Technologies

- **Framework**: [Docusaurus 3.9.2](https://docusaurus.io/)
- **Rendering**: [React 18](https://react.dev/)
- **Styling**: [Tailwind CSS](https://tailwindcss.com/) + CSS Modules
- **Language**: [TypeScript](https://www.typescriptlang.org/)
- **Authentication**: React Hooks + localStorage

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines on:
- How to add new chapters
- Code example standards
- Review process
- Development workflow

## License

This work is licensed under the **MIT License** - see [LICENSE](LICENSE) file for details.

You are free to:
- Use this content for educational purposes
- Share and modify the content
- Use it in commercial projects
- Distribute derivatives

Just provide attribution to the original authors.

## Writing Standards

### Chapters
- 2,500-3,500 words per chapter
- Include practical examples
- Use clear headings and sections
- Add diagrams where helpful

### Code Examples
- Include language identifier in code blocks
- Provide explanatory comments
- Show input and output
- Link to full source if available

### Images & Assets
- Use descriptive file names
- Keep file sizes optimized
- Include alt text for accessibility
- Store in `static/images/`

## Building & Deployment

### Local Testing
```bash
npm run build
npm run serve  # Test production build locally
```

### GitHub Pages Deployment
```bash
npm run deploy
```

### Docker Deployment
```bash
docker build -t physical-ai-book .
docker run -p 3000:3000 physical-ai-book
```

## Related Resources

- **Backend API**: See `../backend/` for FastAPI server
- **Project Spec**: See `../specs/001-mvp-features/` for requirements
- **Architecture**: See `../specs/001-mvp-features/plan.md` for design
- **Main Repository**: [Physical-AI-Humanoid-Robotics](https://github.com/SHAJAR5110/Physical-AI-Humanoid-Robotics-Book)

## Getting Help

- **Issues**: Report bugs or request features on GitHub
- **Discussions**: Ask questions in GitHub Discussions
- **Documentation**: Check existing chapters for answers
- **Examples**: See code examples in each chapter

## Acknowledgments

Built as part of the Physical AI & Humanoid Robotics project, following **Spec-Driven Development** principles for clarity and quality.

---

**Last Updated**: December 2025
**Status**: Open Source | Active Development
**License**: MIT
