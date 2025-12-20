// Physical AI Book RAG Chatbot Widget v3
// Features: Text selection, RAG with Qdrant, chat history
console.log('🤖 Loading Physical AI Chatbot v3...');

window.addEventListener('load', function() {
    console.log('✅ Initializing chatbot...');

    // Configuration
    // Chatbot RAG API - Points to RAG backend server
    const API_URL = window.location.hostname === 'localhost'
        ? 'http://localhost:8003'
        : 'https://physical-ai-book.vercel.app/api/rag';

    let selectedText = '';
    let sessionId = localStorage.getItem('chatbot_session_id') || null;

    // Create widget HTML
    const widget = document.createElement('div');
    widget.id = 'ai-chatbot-widget';
    widget.style.cssText = 'position: fixed; bottom: 20px; right: 20px; z-index: 9999;';

    widget.innerHTML = `
        <button id="chat-toggle" style="
            width: 60px;
            height: 60px;
            border-radius: 50%;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border: none;
            cursor: pointer;
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
            color: white;
            font-size: 24px;
            transition: transform 0.2s;
        ">💬</button>

        <div id="chat-window" style="
            display: none;
            position: absolute;
            bottom: 80px;
            right: 0;
            width: 400px;
            height: 600px;
            background: white;
            border-radius: 16px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.2);
            flex-direction: column;
            overflow: hidden;
        ">
            <!-- Header -->
            <div style="
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                padding: 16px;
                color: white;
                font-weight: 600;
                display: flex;
                justify-content: space-between;
                align-items: center;
            ">
                <div>
                    <div style="font-size: 16px;">🤖 Physical AI Assistant</div>
                    <div style="font-size: 11px; opacity: 0.9; margin-top: 2px;">RAG-powered with Qdrant</div>
                </div>
                <button id="chat-close" style="
                    background: none;
                    border: none;
                    color: white;
                    font-size: 24px;
                    cursor: pointer;
                    padding: 0;
                    width: 30px;
                    height: 30px;
                ">×</button>
            </div>

            <!-- Selected Text Display -->
            <div id="selected-text-display" style="
                display: none;
                background: #fff3cd;
                padding: 12px;
                border-bottom: 1px solid #ffc107;
                font-size: 13px;
            ">
                <div style="display: flex; justify-content: space-between; align-items: start;">
                    <div>
                        <strong>📌 Selected Text:</strong>
                        <div id="selected-text-content" style="
                            margin-top: 6px;
                            font-style: italic;
                            max-height: 60px;
                            overflow-y: auto;
                            color: #666;
                        "></div>
                    </div>
                    <button id="clear-selection" style="
                        background: none;
                        border: none;
                        cursor: pointer;
                        font-size: 20px;
                        padding: 0;
                        color: #666;
                    ">×</button>
                </div>
            </div>

            <!-- Messages -->
            <div id="chat-messages" style="
                flex: 1;
                overflow-y: auto;
                padding: 16px;
                background: #f9f9f9;
            "></div>

            <!-- Input -->
            <div style="
                padding: 16px;
                background: white;
                border-top: 1px solid #eee;
            ">
                <div style="display: flex; gap: 8px; margin-bottom: 8px;">
                    <button id="select-text-btn" style="
                        padding: 6px 12px;
                        background: #e9ecef;
                        border: 1px solid #ddd;
                        border-radius: 6px;
                        cursor: pointer;
                        font-size: 12px;
                        color: #495057;
                    ">📝 Select Text</button>
                    <button id="clear-chat-btn" style="
                        padding: 6px 12px;
                        background: #e9ecef;
                        border: 1px solid #ddd;
                        border-radius: 6px;
                        cursor: pointer;
                        font-size: 12px;
                        color: #495057;
                    ">🗑️ Clear</button>
                </div>
                <div style="display: flex; gap: 8px;">
                    <input
                        id="chat-input"
                        type="text"
                        placeholder="Ask about the book..."
                        style="
                            flex: 1;
                            padding: 12px;
                            border: 1px solid #ddd;
                            border-radius: 8px;
                            font-size: 14px;
                            outline: none;
                        "
                    />
                    <button id="chat-send" style="
                        padding: 12px 24px;
                        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                        color: white;
                        border: none;
                        border-radius: 8px;
                        cursor: pointer;
                        font-weight: 600;
                        font-size: 14px;
                    ">Send</button>
                </div>
            </div>
        </div>
    `;

    document.body.appendChild(widget);

    // Get elements
    const toggleBtn = document.getElementById('chat-toggle');
    const closeBtn = document.getElementById('chat-close');
    const chatWindow = document.getElementById('chat-window');
    const sendBtn = document.getElementById('chat-send');
    const input = document.getElementById('chat-input');
    const messagesDiv = document.getElementById('chat-messages');
    const selectTextBtn = document.getElementById('select-text-btn');
    const clearChatBtn = document.getElementById('clear-chat-btn');
    const selectedTextDisplay = document.getElementById('selected-text-display');
    const selectedTextContent = document.getElementById('selected-text-content');
    const clearSelectionBtn = document.getElementById('clear-selection');

    // Welcome message
    addMessage('bot', 'Hello! 👋 I\'m your AI assistant for the Physical AI and Humanoid Robotics book.\n\nYou can:\n• Ask me questions about any topic\n• Select text and click "📝 Select Text" to ask about it\n• Get answers backed by the book\'s content');

    // Event Listeners
    toggleBtn.onclick = function() {
        const isVisible = chatWindow.style.display === 'flex';
        chatWindow.style.display = isVisible ? 'none' : 'flex';
        toggleBtn.style.transform = isVisible ? 'scale(1)' : 'scale(1.1)';
    };

    closeBtn.onclick = function() {
        chatWindow.style.display = 'none';
        toggleBtn.style.transform = 'scale(1)';
    };

    sendBtn.onclick = sendMessage;

    input.onkeypress = function(e) {
        if (e.key === 'Enter') sendMessage();
    };

    selectTextBtn.onclick = captureSelectedText;

    clearChatBtn.onclick = function() {
        if (confirm('Clear chat history?')) {
            messagesDiv.innerHTML = '';
            addMessage('bot', 'Chat cleared! How can I help you?');
        }
    };

    clearSelectionBtn.onclick = clearSelectedText;

    // Hover effects
    toggleBtn.onmouseenter = () => toggleBtn.style.transform = 'scale(1.1)';
    toggleBtn.onmouseleave = () => {
        if (chatWindow.style.display !== 'flex') {
            toggleBtn.style.transform = 'scale(1)';
        }
    };

    // Functions
    function captureSelectedText() {
        const selection = window.getSelection();
        const text = selection.toString().trim();

        if (text) {
            selectedText = text;
            selectedTextContent.textContent = text.length > 150
                ? text.substring(0, 150) + '...'
                : text;
            selectedTextDisplay.style.display = 'block';
            input.placeholder = 'Ask about the selected text...';
            addMessage('system', `✅ Text selected! (${text.length} characters)\nNow ask me a question about it.`);
        } else {
            alert('Please select some text from the book first!');
        }
    }

    function clearSelectedText() {
        selectedText = '';
        selectedTextDisplay.style.display = 'none';
        input.placeholder = 'Ask about the book...';
    }

    async function sendMessage() {
        const question = input.value.trim();
        if (!question) return;

        console.log('📤 Sending question:', question);

        // Add user message
        addMessage('user', question);
        input.value = '';

        // Show typing indicator
        const typingId = addMessage('bot', '...');

        try {
            const response = await fetch(`${API_URL}/query`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    question: question,
                    selected_text: selectedText,
                    session_id: sessionId
                })
            });

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const data = await response.json();

            // Save session ID
            if (data.session_id) {
                sessionId = data.session_id;
                localStorage.setItem('chatbot_session_id', sessionId);
            }

            // Remove typing indicator
            document.getElementById(typingId)?.remove();

            // Add answer
            let answer = data.answer;

            // Add sources
            if (data.sources && data.sources.length > 0) {
                answer += '\n\n📚 **Sources:**\n' + data.sources.map(s =>
                    `• ${s.replace('.md', '')}`
                ).join('\n');
            }

            addMessage('bot', answer);

            // Clear selected text after answering
            if (selectedText) {
                clearSelectedText();
            }

            console.log('✅ Response received');

        } catch (error) {
            console.error('❌ Error:', error);
            document.getElementById(typingId)?.remove();
            addMessage('bot', `❌ Error: ${error.message}\n\nPlease make sure the backend is running.`);
        }
    }

    function removeMessage(messageId) {
        const messageDiv = document.getElementById(messageId);
        if (messageDiv) {
            messageDiv.remove();
        }
    }

    function addMessage(sender, text) {
        const messageId = 'msg-' + Date.now() + '-' + Math.random();
        const isBot = sender === 'bot';
        const isSystem = sender === 'system';

        const messageDiv = document.createElement('div');
        messageDiv.id = messageId;
        messageDiv.style.cssText = `
            margin-bottom: 12px;
            display: flex;
            justify-content: ${isBot || isSystem ? 'flex-start' : 'flex-end'};
        `;

        const bubble = document.createElement('div');
        bubble.style.cssText = `
            max-width: 80%;
            padding: 12px 16px;
            border-radius: 12px;
            background: ${
                isSystem ? '#e3f2fd' :
                isBot ? '#e9ecef' :
                'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'
            };
            color: ${isBot || isSystem ? '#333' : 'white'};
            font-size: 14px;
            line-height: 1.5;
            white-space: pre-wrap;
            word-wrap: break-word;
        `;

        // Parse markdown-style bold
        const formattedText = text.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
        bubble.innerHTML = formattedText;

        messageDiv.appendChild(bubble);
        messagesDiv.appendChild(messageDiv);
        messagesDiv.scrollTop = messagesDiv.scrollHeight;

        return messageId;
    }

    console.log('✅ Chatbot ready!');
});
