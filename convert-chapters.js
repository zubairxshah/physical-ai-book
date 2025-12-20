const fs = require('fs');
const path = require('path');

const chapters = [5, 6, 7, 8, 9, 10, 11, 12];

for (const chapterNum of chapters) {
  const mdPath = path.join(__dirname, 'docs', `chapter${chapterNum}.md`);
  const mdxPath = path.join(__dirname, 'docs', `chapter${chapterNum}.mdx`);

  try {
    // Read the markdown file
    const content = fs.readFileSync(mdPath, 'utf8');

    // Split into frontmatter and content
    const parts = content.split(/^---$/gm);
    const frontmatter = parts[1] ? parts[1].trim() : '';
    const mainContent = parts[2] ? parts[2].trim() : content;

    // Create MDX content with ProtectedContent wrapper
    const mdxContent = `---
${frontmatter}---

import ProtectedContent from '@site/src/components/ProtectedContent';

<ProtectedContent chapterId="chapter${chapterNum}">

${mainContent.trim()}

</ProtectedContent>
`;

    // Write the MDX file
    fs.writeFileSync(mdxPath, mdxContent);
    console.log(`✅ Converted chapter${chapterNum}.md to chapter${chapterNum}.mdx`);

  } catch (error) {
    console.error(`❌ Error converting chapter${chapterNum}:`, error.message);
  }
}

console.log('\n✨ Conversion complete!');
