// Chatbot Widget - Simple Version
console.log('🤖 Loading chatbot widget...');

// Wait for everything to be ready
window.addEventListener('load', function() {
    console.log('✅ Window loaded, initializing chatbot...');
    
    // Configuration
    const API_URL = 'http://localhost:8000';
    let selectedText = '';
    
    // Create widget
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
        ">💬</button>
        
        <div id="chat-window" style="
            display: none;
            position: absolute;
            bottom: 80px;
            right: 0;
            width: 380px;
            height: 550px;
            background: white;
            border-radius: 12px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.2);
            flex-direction: column;
        ">
            <div style="
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                padding: 16px;
                color: white;
                font-weight: 600;
                display: flex;
                justify-content: space-between;
            ">
                <span>🤖 Physical AI Assistant</span>
                <button id="chat-close" style="
                    background: none;
                    border: none;
                    color: white;
                    font-size: 20px;
                    cursor: pointer;
                ">✕</button>
            </div>
            
            <div id="chat-messages" style="
                flex: 1;
                overflow-y: auto;
                padding: 16px;
                background: #f9f9f9;
            "></div>
            
            <div style="padding: 16px; background: white; border-top: 1px solid #eee;">
                <div style="display: flex; gap: 8px;">
                    <input 
                        id="chat-input" 
                        type="text" 
                        placeholder="Ask about the book..."
                        style="
                            flex: 1;
                            padding: 10px;
                            border: 1px solid #ddd;
                            border-radius: 6px;
                            font-size: 14px;
                        "
                    />
                    <button id="chat-send" style="
                        padding: 10px 20px;
                        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                        color: white;
                        border: none;
                        border-radius: 6px;
                        cursor: pointer;
                        font-weight: 600;
                    ">Send</button>
                </div>
            </div>
        </div>
    `;
    
    document.body.appendChild(widget);
    console.log('✅ Widget HTML added to page');
    
    // Get elements
    const toggleBtn = document.getElementById('chat-toggle');
    const closeBtn = document.getElementById('chat-close');
    const chatWindow = document.getElementById('chat-window');
    const sendBtn = document.getElementById('chat-send');
    const input = document.getElementById('chat-input');
    const messagesDiv = document.getElementById('chat-messages');
    
    console.log('Elements:', {
        toggleBtn: !!toggleBtn,
        closeBtn: !!closeBtn,
        chatWindow: !!chatWindow,
        sendBtn: !!sendBtn,
        input: !!input,
        messagesDiv: !!messagesDiv
    });
    
    // Add welcome message
    addMessage('bot', 'Hello! 👋 Ask me anything about Physical AI and Humanoid Robotics!');
    
    // Toggle button click
    toggleBtn.onclick = function() {
        console.log('🖱️ Toggle button clicked!');
        if (chatWindow.style.display === 'none') {
            chatWindow.style.display = 'flex';
            console.log('✅ Chat opened');
        } else {
            chatWindow.style.display = 'none';
            console.log('✅ Chat closed');
        }
    };
    
    // Close button click
    closeBtn.onclick = function() {
        console.log('🖱️ Close button clicked!');
        chatWindow.style.display = 'none';
    };
    
    // Send button click
    sendBtn.onclick = sendMessage;
    
    // Enter key
    input.onkeypress = function(e) {
        if (e.key === 'Enter') {
            sendMessage();
        }
    };
    
    async function sendMessage() {
        const question = input.value.trim();
        if (!question) return;
        
        console.log('📤 Sending message:', question);
        
        addMessage('user', question);
        input.value = '';
        
        const typingId = addMessage('bot', '...');
        
        try {
            const response = await fetch(API_URL + '/query', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ question: question, selected_text: '' })
            });
            
            const data = await response.json();
            
            // Remove typing
            document.getElementById(typingId).remove();
            
            // Add answer
            let answer = data.answer;
            if (data.sources && data.sources.length > 0) {
                answer += '\n\n📚 Sources: ' + data.sources.join(', ');
            }
            addMessage('bot', answer);
            
            console.log('✅ Got response');
            
        } catch (error) {
            console.error('❌ Error:', error);
            document.getElementById(typingId).remove();
            addMessage('bot', '❌ Error connecting to backend. Make sure it\'s running on ' + API_URL);
        }
    }
    
    function addMessage(sender, text) {
        const messageId = 'msg-' + Date.now();
        const isBot = sender === 'bot';
        
        const messageDiv = document.createElement('div');
        messageDiv.id = messageId;
        messageDiv.style.cssText = `
            margin-bottom: 12px;
            display: flex;
            justify-content: ${isBot ? 'flex-start' : 'flex-end'};
        `;
        
        const bubble = document.createElement('div');
        bubble.style.cssText = `
            max-width: 75%;
            padding: 10px 14px;
            border-radius: 12px;
            background: ${isBot ? '#e9ecef' : 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)'};
            color: ${isBot ? '#333' : 'white'};
            font-size: 14px;
            line-height: 1.4;
            white-space: pre-wrap;
        `;
        bubble.textContent = text;
        
        messageDiv.appendChild(bubble);
        messagesDiv.appendChild(messageDiv);
        messagesDiv.scrollTop = messagesDiv.scrollHeight;
        
        return messageId;
    }
    
    console.log('✅ Chatbot ready! Click the 💬 button to start.');
});
