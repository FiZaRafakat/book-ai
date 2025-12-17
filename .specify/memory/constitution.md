<!--
SYNC IMPACT REPORT
Version change: N/A -> 1.0.0 (initial creation)
Added sections: All principles and governance sections
Removed sections: None
Templates requiring updates: ✅ updated / ⚠ pending
  - .specify/templates/plan-template.md ⚠ pending
  - .specify/templates/spec-template.md ⚠ pending
  - .specify/templates/tasks-template.md ⚠ pending
Follow-up TODOs: None
-->

# Book-AI Constitution
<!-- Spec-Driven Technical Book with Embedded RAG Chatbot -->

## Core Principles

### I. Spec-First, Deterministic Execution
All development begins with a formally specified requirement document using Spec-Kit Plus; Implementation must strictly adhere to approved specifications without deviation; Every feature and behavior must be traceable to an approved specification.

### II. Technical Accuracy and Verifiability
All content and code examples must be technically accurate and verifiable; No speculative or unverified information allowed; All claims must be backed by authoritative sources or reproducible evidence.

### III. Developer-Focused Clarity (NON-NEGOTIABLE)
Content must be written for intermediate-advanced developers with clear, practical examples; Complex concepts broken down into digestible, actionable insights; All code samples tested and validated in real environments.

### IV. Zero Hallucination Tolerance
The RAG chatbot must only respond with information derived from the book content; No fabricated or hallucinated responses allowed; Strict adherence to source material is mandatory.

### V. Full Reproducibility
All examples, configurations, and deployment procedures must be fully reproducible; Detailed step-by-step instructions with expected outcomes; Code and infrastructure as documented.

### VI. Docusaurus-Compatible Structure

All book content must conform to Docusaurus publishing standards; Proper markdown formatting and navigation structure maintained; SEO and accessibility considerations implemented.

## Technical Standards

All book content generated from approved specs; Audience is intermediate–advanced developers; Docusaurus-compatible structure with no filler or speculative content; Implementation follows Spec-Kit Plus methodology with clear scope, constraints, and acceptance criteria.

## Development Workflow

Implementation must not exceed spec scope; RAG chatbot stack uses OpenAI Agents/ChatKit SDK and FastAPI; Storage utilizes Qdrant Cloud (Free Tier) and Neon Serverless Postgres; Data source is book content only supporting full-book Q&A and selected-text-only Q&A.

## Governance

This constitution supersedes all other practices for the Book-AI project; All PRs and reviews must verify compliance with spec-first methodology and technical accuracy requirements; Amendments require formal documentation and team approval with clear justification.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17
