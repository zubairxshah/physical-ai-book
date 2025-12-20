<!--
Sync Impact Report:
Version change: INITIAL → 1.0.0
Modified principles: N/A (initial creation)
Added sections: All sections (initial creation)
Removed sections: N/A
Templates requiring updates:
  ✅ spec-template.md - verified (already compliant)
  ✅ plan-template.md - verified (already compliant)
  ✅ tasks-template.md - verified (already compliant)
Follow-up TODOs: None
-->

# Physical AI Book Project Constitution

## Core Principles

### I. Content Quality and Accuracy

The Physical AI Book must maintain the highest standards of technical accuracy and educational value. All content MUST be:
- Technically accurate and up-to-date with current industry practices
- Supported by working, tested code examples
- Written clearly for the target audience (AI/robotics learners)
- Cross-referenced with authoritative sources where appropriate
- Reviewed for consistency across all chapters and modules

**Rationale**: As an educational resource for GIAIC students and the broader AI community, accuracy is non-negotiable. Incorrect information undermines trust and learning outcomes. Working code examples demonstrate concepts practically and build student confidence.

### II. Spec-Driven Development

All features and content additions MUST follow the spec-driven development workflow:
- Requirements specified BEFORE implementation
- Architecture planned BEFORE coding
- Tasks broken down BEFORE execution
- User stories MUST be independently testable
- Each specification MUST have measurable success criteria

**Rationale**: Spec-driven development ensures clarity of intent, enables parallel work, reduces rework, and maintains project coherence. This approach is especially critical for educational materials where structure directly impacts learning effectiveness.

### III. RAG Chatbot Excellence

The RAG chatbot MUST provide accurate, contextual answers with proper source attribution:
- Vector embeddings MUST be regenerated when content changes significantly
- Responses MUST include chapter/section citations
- Query accuracy MUST be validated through testing
- Text selection feature MUST work reliably across all devices
- Performance MUST meet user expectations (<3s response time for queries)

**Rationale**: The chatbot is a core differentiator and learning enhancement tool. Poor chatbot performance degrades the entire user experience and reduces the project's value proposition for the hackathon.

### IV. Documentation and Knowledge Preservation

All project knowledge MUST be documented for continuity:
- `PROJECT_MEMORY.md` MUST be updated after significant sessions
- Architecture decisions MUST be recorded as ADRs when significant
- Setup procedures MUST be documented in quickstart guides
- Environment variables MUST have `.env.example` templates
- Git commits MUST have clear, descriptive messages

**Rationale**: This project was built incrementally across multiple sessions. Without comprehensive documentation, context is lost, leading to redundant work, inconsistent decisions, and deployment failures.

### V. Deployment Reliability

Deployment configurations MUST be maintained and validated:
- All environment variables MUST be documented
- Backend API endpoints MUST be tested before frontend integration
- Vercel/serverless configurations MUST be version-controlled
- Health checks MUST be implemented for all services
- Database migrations MUST be reversible

**Rationale**: A deployed, working project is the ultimate deliverable. Deployment failures waste time, reduce hackathon scores, and frustrate users. Production reliability is a quality signal.

### VI. Code Simplicity and Maintainability

Code MUST prioritize clarity and simplicity:
- Avoid premature optimization
- Use clear, descriptive variable and function names
- Keep functions focused and under 50 lines where practical
- Avoid unnecessary abstractions
- Comment only when logic is non-obvious
- Delete unused code completely (no commented-out code)

**Rationale**: This project may be reviewed by evaluators, forked by students, or maintained by future contributors. Simple, clear code communicates intent and reduces cognitive load. Over-engineering signals inexperience, not sophistication.

### VII. Test Coverage (When Required)

When tests are explicitly required in specifications:
- Tests MUST be written BEFORE implementation (TDD)
- Tests MUST fail initially to validate they test the right thing
- Each user story MUST have independent test coverage
- Contract tests MUST verify API boundaries
- Integration tests MUST validate end-to-end workflows

**Rationale**: TDD ensures testability by design and catches regressions early. However, not all features require tests—testing should serve the specification, not be cargo-culted. Educational projects prioritize working demonstrations over exhaustive test suites.

## Technology Standards

### Frontend Standards
- **Framework**: Docusaurus 3.6+ (for documentation sites)
- **Language**: TypeScript (preferred) or JavaScript ES6+
- **Styling**: CSS Modules or inline styles (avoid Tailwind unless specified)
- **React Version**: 18+ with functional components and hooks
- **Compatibility**: Modern browsers (Chrome, Firefox, Safari, Edge - last 2 versions)

### Backend Standards
- **Framework**: FastAPI (Python 3.11+) for APIs
- **Database**: Neon Postgres for transactional data
- **Vector Database**: Qdrant Cloud for embeddings
- **AI Models**: OpenRouter API (free tier models) or OpenAI API
- **API Design**: RESTful conventions, JSON responses, proper HTTP status codes
- **Error Handling**: Structured error responses with error codes and messages

### Infrastructure Standards
- **Hosting**: Vercel (primary), GitHub Pages (fallback)
- **Serverless**: Vercel Functions with Mangum adapter for FastAPI
- **Environment Variables**: Never commit secrets; use `.env` files locally
- **Version Control**: Git with clear, conventional commit messages
- **Branching**: Feature branches with descriptive names (`###-feature-name`)

## Development Workflow

### Feature Development Process

1. **Specification Phase** (`/sp.specify`)
   - Create `specs/<feature>/spec.md` with user stories and acceptance criteria
   - Define functional requirements with clear FR-XXX identifiers
   - Establish measurable success criteria
   - Prioritize user stories (P1, P2, P3) for independent implementation

2. **Planning Phase** (`/sp.plan`)
   - Create `specs/<feature>/plan.md` with architecture decisions
   - Verify constitution compliance (gates)
   - Document technical context and dependencies
   - Create research, data models, and API contracts as needed
   - Suggest ADRs for significant architectural decisions

3. **Task Breakdown** (`/sp.tasks`)
   - Create `specs/<feature>/tasks.md` with concrete, actionable tasks
   - Organize tasks by user story for independent execution
   - Mark parallelizable tasks with `[P]`
   - Include exact file paths in task descriptions
   - Ensure each user story can be implemented and tested independently

4. **Implementation**
   - Execute tasks in priority order (P1 → P2 → P3)
   - Validate each user story independently before proceeding
   - Commit frequently with descriptive messages
   - Update documentation as changes are made

5. **Knowledge Capture** (Prompt History Records)
   - Create PHRs for significant interactions after completion
   - Route PHRs to appropriate subdirectories (`history/prompts/constitution/`, `history/prompts/<feature>/`, or `history/prompts/general/`)
   - Capture full user input and representative assistant output
   - Include all context (files modified, decisions made, outcomes)

### Git Workflow

- **Commit Messages**: Use conventional commit format (`feat:`, `fix:`, `docs:`, `refactor:`, `chore:`)
- **Commit Frequency**: Commit after completing logical units of work (individual tasks or small groups)
- **Branch Naming**: Feature branches use `###-feature-name` format
- **Pull Requests**: Required for significant features; include description and testing notes
- **Co-Authoring**: AI-assisted commits include Claude co-author attribution

### Architecture Decision Records (ADRs)

ADRs MUST be created when decisions meet ALL three criteria:
1. **Impact**: Long-term consequences affecting system design, data models, APIs, security, or platform choices
2. **Alternatives**: Multiple viable options were considered with non-trivial tradeoffs
3. **Scope**: Decision is cross-cutting and influences multiple components or future development

**When to suggest ADRs** (require user consent before creating):
- Choosing frameworks or major libraries (Docusaurus, FastAPI, Qdrant)
- API design patterns or authentication mechanisms
- Data model or database schema decisions
- Deployment architecture or hosting platform selection
- RAG implementation approach (embeddings, chunking strategy, retrieval methods)

**When NOT to create ADRs**:
- Tactical implementation choices within a single component
- Trivial decisions easily reversed (e.g., variable naming, file organization)
- Following established project patterns (no new decision being made)

## Quality Gates

### Pre-Implementation Gates
- [ ] Specification includes prioritized, independently testable user stories
- [ ] Functional requirements are clear and unambiguous
- [ ] Success criteria are measurable and verifiable
- [ ] Technical context is documented in plan
- [ ] Constitution compliance verified (no unjustified violations)

### Pre-Deployment Gates
- [ ] All P1 user stories implemented and tested
- [ ] Environment variables documented in `.env.example`
- [ ] Health check endpoints functional
- [ ] Frontend build succeeds without errors
- [ ] Backend API endpoints tested (manually or via curl)
- [ ] RAG chatbot returns accurate answers with sources (if applicable)

### Post-Deployment Gates
- [ ] Production site loads without errors
- [ ] Chatbot functional on production environment
- [ ] Database connections verified (Neon, Qdrant)
- [ ] No secrets exposed in client-side code
- [ ] PROJECT_MEMORY.md updated with deployment details

## Security Requirements

- **API Keys**: NEVER commit to Git; use environment variables exclusively
- **Input Validation**: Validate and sanitize all user inputs in backend
- **CORS**: Configure CORS properly for production domains
- **Rate Limiting**: Implement rate limiting on public API endpoints
- **Database Access**: Use parameterized queries; never string concatenation
- **Authentication**: If implementing auth, use industry-standard libraries (OAuth, JWT)
- **Secrets Management**: Use Vercel environment variables for production secrets

## Performance Standards

- **Page Load**: First contentful paint <2s on 3G connections
- **Chatbot Response**: Query responses <3s for typical questions
- **Build Time**: Frontend build completes in <3 minutes
- **Bundle Size**: JavaScript bundles <500KB (gzipped) for main chunk
- **Vector Search**: Qdrant queries return results in <500ms
- **Database Queries**: Postgres queries <200ms for simple reads

## Governance

### Amendment Process
1. Proposed changes to this constitution MUST be documented
2. Version number MUST be incremented according to semantic versioning:
   - **MAJOR**: Backward-incompatible changes (e.g., removing principles, changing requirements)
   - **MINOR**: New principles or sections added
   - **PATCH**: Clarifications, typo fixes, non-semantic refinements
3. Sync Impact Report MUST be generated as HTML comment at top of file
4. Dependent templates (spec, plan, tasks) MUST be reviewed and updated if affected
5. ADR should be created for significant constitutional amendments

### Compliance Verification
- All PRs MUST verify compliance with applicable principles
- Complexity (violations of simplicity principle) MUST be justified in plan.md
- Unjustified violations MUST be remediated before merging
- Constitution supersedes all other practices except explicit user directives

### Runtime Guidance
- Agents and developers MUST consult `CLAUDE.md` for execution guidance
- `PROJECT_MEMORY.md` provides session continuity and current project state
- This constitution provides non-negotiable constraints and quality standards
- When guidance conflicts, order of precedence: User directive > Constitution > CLAUDE.md > PROJECT_MEMORY.md

**Version**: 1.0.0 | **Ratified**: 2025-12-19 | **Last Amended**: 2025-12-19
