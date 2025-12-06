// Chatbot Widget for Physical AI Book
(function() {
    // Configuration - CHANGE THIS to your deployed API URL
    const API_URL = 'http://localhost:8000';
    
    // State
    let selectedText = '';
    
    // Create widget HTML
    const widgetHTML = `
        <div id="ai-chatbot-widget" style="position: fixed; bottom: 20px; right: 20px; z-index: 9999;">
            <button id="chat-toggle" style="
                width: 60px;
                height: 60px;
                border-radius: 50%;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                border: none;
                cursor: pointer;
                box-shadow: 0 4px 12px rgba(0,0,0,0.15);
                display: flex;
                align-items: center;
                justify-content: center;
                color: white;
                font-size: 24px;
                transition: transform 0.2s;
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
                overflow: hidden;
                flex-direction: column;
            ">
                <div style="
                    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                    padding: 16px;
                    color: white;
                    font-weight: 600;
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                ">
                    <span>🤖 Physical AI Assistant</span>
                    <button id="chat-close" style="
                        background: none;
                        border: none;
                        color: white;
                        font-size: 20px;
                        cursor: pointer;
                        padding: 0;
                        width: 24px;
                        height: 24px;
                    ">✕</button>
                </div>
                
                <div id="selected-text-display" style="
                    display: none;
                    background: #f0f0f0;
                    padding: 8px 12px;
                    border-bottom: 1px solid #ddd;
                    font-size: 12px;
                    color: #666;
                ">
                    <div style="display: flex; justify-content: space-between; align-items: center;">
                        <strong>Selected text:</strong>
                        <button id="clear-selection" style="
                            background: none;
                            border: none;
                            color: #667eea;
                            cursor: pointer;
                            font-size: 16px;
                            padding: 0;
                        ">✕</button>
                    </div>
                    <div id="selected-text-content" style="
                        margin-top: 4px;
                        font-style: italic;
                        max-height: 60px;
                        overflow-y: auto;
                    "></div>
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
                                outline: none;
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
                            transition: opacity 0.2s;
                        ">Send</button>
                    </div>
                </div>
            </div>
        </div>
    `;
    
    // Initialize when DOM is ready
    console.log('Chatbot widget script loaded, DOM state:', document.readyState);
    
    if (document.readyState === 'loading') {
        console.log('Waiting for DOMContentLoaded...');
        document.addEventListener('DOMContentLoaded', init);
    } else {
        console.log('DOM already loaded, initializing immediately...');
        init();
    }
    
    function init() {
        console.log('Initializing chatbot widget...');
        try {
            document.body.insertAdjacentHTML('beforeend', widgetHTML);
            console.log('Widget HTML inserted');
            initChatbot();
            console.log('✅ Chatbot initialized successfully! Click the 💬 button.');
        } catch (error) {
            console.error('❌ Error initializing chatbot:', error);
        }
    }
    
    function initChatbot() {
        console.log('Setting up chatbot event listeners...');
        
        const toggleBtn = document.getElementById('chat-toggle');
        const closeBtn = document.getElementById('chat-close');
        const chatWindow = document.getElementById('chat-window');
        const sendBtn = document.getElementById('chat-send');
        const input = document.getElementById('chat-input');
        const clearSelectionBtn = document.getElementById('clear-selection');
        
        console.log('Elements found:', {
            toggleBtn: !!toggleBtn,
            closeBtn: !!closeBtn,
            chatWindow: !!chatWindow,
            sendBtn: !!sendBtn,
            input: !!input,
            clearSelectionBtn: !!clearSelectionBtn
        });
        
        if (!toggleBtn || !closeBtn || !chatWindow || !sendBtn || !input || !clearSelectionBtn) {
            console.error('❌ Chatbot: Required elements not found');
            return;
        }
        
        // Toggle chat window
        toggleBtn.addEventListener('click', function() {
            console.log('Chat toggle clicked');
            const isHidden = chatWindow.style.display === 'none';
            chatWindow.style.display = isHidden ? 'flex' : 'none';
            toggleBtn.style.transform = isHidden ? 'scale(0.9)' : 'scale(1)';
            console.log('Chat window', isHidden ? 'opened' : 'closed');
        });
        
        closeBtn.addEventListener('click', function() {
            chatWindow.style.display = 'none';
            toggleBtn.style.transform = 'scale(1)';
        });
        
        // Send message handlers
        sendBtn.addEventListener('click', sendMessage);
        input.addEventListener('keypress', function(e) {
            if (e.key === 'Enter') sendMessage();
        });
        
        // Clear selection
        clearSelectionBtn.addEventListener('click', function() {
            selectedText = '';
            const display = document.getElementById('selected-text-display');
            if (display) display.style.display = 'none';
        });
        
        // Text selection feature
        document.addEventListener('mouseup', handleTextSelection);
        
        // Welcome message
        addMessage('bot', 'Hello! 👋 I can answer questions about Physical AI and Humanoid Robotics.\n\nTry:\n• Asking general questions\n• Selecting text and asking about it\n• Exploring different chapters!');
    }
    
    function handleTextSelection() {
        const selection = window.getSelection();
        if (!selection) return;
        
        const text = selection.toString().trim();
        
        if (text.length > 10) {
            selectedText = text;
            const display = document.getElementById('selected-text-display');
            const content = document.getElementById('selected-text-content');
            
            if (display && content) {
                display.style.display = 'block';
                content.textContent = text.length > 100 ? text.substring(0, 100) + '...' : text;
            }
        }
    }
    
    async function sendMessage() {
        const input = document.getElementById('chat-input');
        const sendBtn = document.getElementById('chat-send');
        
        if (!input || !sendBtn) return;
        
        const question = input.value.trim();
        if (!question) return;
        
        // Disable input while processing
        input.disabled = true;
        sendBtn.disabled = true;
        sendBtn.style.opacity = '0.6';
        
        // Add user message
        addMessage('user', question);
        input.value = '';
        
        // Show typing indicator
        const typingId = addMessage('bot', '...', true);
        
        try {
            const response = await fetch(API_URL + '/query', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    question: question,
                    selected_text: selectedText
                })
            });
            
            if (!response.ok) {
                throw new Error('Failed to get response');
            }
            
            const data = await response.json();
            
            // Remove typing indicator
            removeMessage(typingId);
            
            // Add bot response
            let botMessage = data.answer;
            if (data.sources && data.sources.length > 0) {
                botMessage += '\n\n📚 Sources: ' + data.sources.join(', ');
            }
            addMessage('bot', botMessage);
            
            // Clear selected text after using it
            if (selectedText) {
                selectedText = '';
                const display = document.getElementById('selected-text-display');
                if (display) display.style.display = 'none';
            }
            
        } catch (error) {
            console.error('Error:', error);
            removeMessage(typingId);
            addMessage('bot', '❌ Sorry, I encountered an error. Please try again.');
        } finally {
            input.disabled = false;
            sendBtn.disabled = false;
            sendBtn.style.opacity = '1';
        }
    }
    
    function addMessage(sender, text, isTyping) {
        const messagesDiv = document.getElementById('chat-messages');
        if (!messagesDiv) return null;
        
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
            word-wrap: break-word;
            ${isTyping ? 'animation: pulse 1.5s infinite;' : ''}
        `;
        bubble.textContent = text;
        
        messageDiv.appendChild(bubble);
        messagesDiv.appendChild(messageDiv);
        messagesDiv.scrollTop = messagesDiv.scrollHeight;
        
        return messageId;
    }
    
    function removeMessage(messageId) {
        if (!messageId) return;
        const msg = document.getElementById(messageId);
        if (msg) msg.remove();
    }
    
    // Add pulse animation for typing indicator
    const style = document.createElement('style');
    style.textContent = `
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        #chat-toggle:hover {
            transform: scale(1.05) !important;
        }
        #chat-send:hover:not(:disabled) {
            opacity: 0.9 !important;
        }
        #chat-input:focus {
            border-color: #667eea !important;
        }
        #chat-messages::-webkit-scrollbar {
            width: 6px;
        }
        #chat-messages::-webkit-scrollbar-track {
            background: #f1f1f1;
            border-radius: 10px;
        }
        #chat-messages::-webkit-scrollbar-thumb {
            background: #888;
            border-radius: 10px;
        }
        #chat-messages::-webkit-scrollbar-thumb:hover {
            background: #555;
        }
    `;
    document.head.appendChild(style);
})();
