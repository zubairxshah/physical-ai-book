# Physical AI and Humanoid Robotics
### Interactive AI-Powered Book with RAG Chatbot

🤖 **Live Book**: [Your GitHub Pages URL]  
🔗 **API Backend**: [Your Render/Railway URL]  
📹 **Demo Video**: [Your video link]

---

## 🎯 Project Overview

This project was built for the GIAIC AI/Spec-Driven Online Hackathon. It demonstrates the future of interactive documentation by combining:

- **AI-Generated Content**: Complete book written using Claude Code and Spec-Kit Plus
- **Modern Documentation**: Built with Docusaurus 3.9
- **Intelligent RAG Chatbot**: Answers questions about book content using OpenAI + Qdrant
- **Text Selection Queries**: Users can highlight text and ask specific questions

---

## 🚀 Features

### Core Features
✅ **Complete Book** - 12 comprehensive chapters on Physical AI and Humanoid Robotics  
✅ **RAG Chatbot** - Retrieval-Augmented Generation using vector database  
✅ **Text Selection** - Highlight any text and ask questions about it  
✅ **Source Citations** - Every answer includes relevant chapter references  
✅ **Responsive Design** - Works on desktop and mobile  

### Technology Stack
- **Frontend**: Docusaurus 3.9, React, TypeScript
- **Backend**: FastAPI, Python 3.11
- **AI/ML**: OpenAI GPT-4, text-embedding-3-small
- **Vector DB**: Qdrant Cloud (free tier)
- **Deployment**: GitHub Pages + Render.com

---

## 📚 Book Contents

1. **Introduction to Physical AI** - What it is and why it matters
2. **Core Technologies** - Computer vision, sensors, real-time systems
3. **AI Models for Physical Systems** - RL, imitation learning, sim-to-real
4. **Rise of Humanoid Robots** - History and current state
5. **Mechanical Design** - Biomechanics, actuators, sensors
6. **Control Systems** - Bipedal walking, manipulation, HRI
7. **Perception** - Vision, tactile sensing, audio processing
8. **Learning and Adaptation** - Self-supervised and continuous learning
9. **Natural Language AI** - Vision-language models, instruction following
10. **Real-World Applications** - Manufacturing, healthcare, domestic use
11. **Challenges** - Safety, cost, technical and ethical issues
12. **Future** - Predictions and emerging directions

---

## 🛠️ Setup Instructions

### Prerequisites
- Node.js 18+ 
- Python 3.11+
- OpenAI API key
- Qdrant Cloud account (free)

### Frontend Setup
```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/physical-ai-book.git
cd physical-ai-book

# Install dependencies
npm install

# Start development server
npm start

# Build and deploy to GitHub Pages
npm run deploy
```

### Backend Setup
```bash
# Navigate to backend folder
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create .env file
echo "OPENAI_API_KEY=your_key_here" > .env
echo "QDRANT_URL=your_qdrant_url" >> .env
echo "QDRANT_API_KEY=your_qdrant_key" >> .env

# Run server
uvicorn main:app --reload
```

---

## 🤖 How the RAG Chatbot Works

1. **Content Ingestion**: All book chapters are embedded using OpenAI's embedding model
2. **Vector Storage**: Embeddings stored in Qdrant Cloud vector database
3. **Query Processing**: User questions are converted to embeddings
4. **Similarity Search**: Most relevant book sections retrieved
5. **Answer Generation**: GPT-4 generates contextual answers with citations
6. **Text Selection**: Highlighted text adds focused context to queries

### Example Queries
- "What is Physical AI?"
- "How do humanoid robots maintain balance?"
- *[Select text about computer vision]* "Explain this in simpler terms"
- "What are the main challenges in robotics?"

---

## 📁 Project Structure

```
physical-ai-book/
├── docs/                    # Book chapters (Markdown)
│   ├── intro.md
│   ├── chapter1.md
│   └── ...
├── src/                     # React components
├── static/
│   └── chatbot-widget.js   # Chatbot UI
├── backend/                 # FastAPI backend
│   ├── main.py             # RAG implementation
│   └── requirements.txt
├── docusaurus.config.js    # Docusaurus configuration
├── package.json
└── README.md
```

---

## 🎥 Demo Video

[Watch the demo](YOUR_VIDEO_LINK) showing:
1. Book navigation and content
2. General chatbot queries
3. Text selection feature
4. Source citations
5. Mobile responsiveness

---

## 🏆 Bonus Features

### Claude Code Integration
- Used Claude Code for rapid content generation
- Spec-driven development approach
- Iterative refinement of chapters

### Subagents & Skills (If implemented)
- Content Generator Subagent
- Documentation Skill
- Research Summarization Skill

---

## 🧪 Testing

The chatbot has been tested with:
- ✅ General knowledge questions
- ✅ Specific technical queries
- ✅ Text selection queries
- ✅ Multi-chapter context
- ✅ Source citation accuracy

---

## 🚀 Deployment

### Frontend (GitHub Pages)
```bash
npm run deploy
```

### Backend (Render.com)
1. Connect GitHub repository
2. Set environment variables
3. Deploy automatically on push

---

## 📊 Evaluation Criteria Met

✅ **AI/Spec-Driven Development** - Built using Claude Code and Spec-Kit Plus  
✅ **Docusaurus Book** - Complete 12-chapter book deployed to GitHub Pages  
✅ **RAG Chatbot** - Fully functional with OpenAI + Qdrant  
✅ **Text Selection** - Highlighted text can be queried  
✅ **Integration** - Chatbot embedded in book website  
✅ **Documentation** - Comprehensive README and code comments  

---

## 👥 Credits

**Student**: [Your Name]  
**Course**: GIAIC Thursday Evening AI  
**Hackathon**: AI/Spec-Driven Online Hackathon 1  
**Date**: December 2024  

**Technologies Used**:
- Claude Code (AI pair programmer)
- Docusaurus (documentation framework)
- OpenAI GPT-4 (chat completion)
- Qdrant (vector database)
- FastAPI (backend framework)

---

## 📝 License

MIT License - feel free to use this code for learning and projects!

---

## 🙏 Acknowledgments

- GIAIC faculty and coordinators
- Anthropic's Claude for development assistance
- OpenAI for GPT-4 and embeddings API
- Qdrant for vector database
- Meta for Docusaurus framework

---

## 📧 Contact

For questions or feedback:
- GitHub: [@zubairxshah]
- Email: engisoft@yahoo.com

---

**Built with ❤️ for GIAIC Hackathon**