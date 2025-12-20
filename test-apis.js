// Simple test script to verify Python APIs are working

async function testPersonalizationAPI() {
  console.log('\n🧪 Testing Personalization API...\n');

  try {
    // Health check
    const healthRes = await fetch('http://localhost:8001/health');
    const healthData = await healthRes.json();
    console.log('✅ Health check:', healthData);

    // Get tooltip
    const tooltipRes = await fetch('http://localhost:8001/tooltips', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        term: 'ROS 2',
        chapter_id: 'test',
        user_profile: {
          programming_experience: 'intermediate',
          ros_familiarity: 'basic',
          ai_ml_background: 'none'
        }
      })
    });
    const tooltipData = await tooltipRes.json();
    console.log('✅ Tooltip response:', tooltipData);

  } catch (error) {
    console.error('❌ Personalization API error:', error.message);
  }
}

async function testTranslationAPI() {
  console.log('\n🧪 Testing Translation API...\n');

  try {
    // Health check
    const healthRes = await fetch('http://localhost:8002/health');
    const healthData = await healthRes.json();
    console.log('✅ Health check:', healthData);

    // Translate content
    const translateRes = await fetch('http://localhost:8002/translate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        chapter_id: 'test',
        content: '# Hello World\n\nThis is a test of the Urdu translation system.'
      })
    });
    const translateData = await translateRes.json();
    console.log('✅ Translation response:');
    console.log('   - Cached:', translateData.cached);
    console.log('   - Tokens used:', translateData.tokens_used);
    console.log('   - First 50 chars:', translateData.translated_content?.substring(0, 50));

  } catch (error) {
    console.error('❌ Translation API error:', error.message);
  }
}

async function main() {
  console.log('🚀 Testing Backend APIs\n');
  console.log('=' .repeat(60));

  await testPersonalizationAPI();
  await testTranslationAPI();

  console.log('\n' + '='.repeat(60));
  console.log('\n✨ API Tests Complete!\n');
}

main();
