const fs = require('fs');
const path = require('path');
const { glob } = require('glob');

/**
 * Generates a JSON index of all glossary terms from markdown files
 * This runs at build time to create a searchable glossary index
 */
async function generateGlossaryIndex() {
  const glossaryDir = path.join(__dirname, '../docs/glossary/terms');
  const outputFile = path.join(__dirname, '../static/glossary-index.json');

  console.log('Generating glossary index...');
  console.log('Glossary directory:', glossaryDir);

  try {
    // Find all markdown files in the glossary terms directory
    const termFiles = await glob('**/*.md', {
      cwd: glossaryDir,
      absolute: true
    });

    console.log(`Found ${termFiles.length} term files`);

    const terms = [];

    for (const filePath of termFiles) {
      try {
        const content = fs.readFileSync(filePath, 'utf-8');
        const term = parseTermFile(content, filePath);

        if (term) {
          terms.push(term);
        }
      } catch (error) {
        console.error(`Error parsing ${filePath}:`, error.message);
      }
    }

    // Sort terms alphabetically
    terms.sort((a, b) => a.term.localeCompare(b.term));

    // Ensure static directory exists
    const staticDir = path.join(__dirname, '../static');
    if (!fs.existsSync(staticDir)) {
      fs.mkdirSync(staticDir, { recursive: true });
    }

    // Write the index
    fs.writeFileSync(outputFile, JSON.stringify(terms, null, 2));

    console.log(`✓ Generated glossary index with ${terms.length} terms`);
    console.log(`✓ Output: ${outputFile}`);

    return terms.length;
  } catch (error) {
    console.error('Failed to generate glossary index:', error);
    throw error;
  }
}

/**
 * Parse a glossary term markdown file
 * @param {string} content - File content
 * @param {string} filePath - File path for reference
 * @returns {Object|null} Parsed term or null if invalid
 */
function parseTermFile(content, filePath) {
  // Extract YAML frontmatter
  const frontMatterMatch = content.match(/^---\n([\s\S]*?)\n---/);
  if (!frontMatterMatch) {
    console.warn(`No frontmatter found in ${filePath}`);
    return null;
  }

  const frontMatter = frontMatterMatch[1];
  const bodyContent = content.slice(frontMatterMatch[0].length).trim();

  // Parse frontmatter fields
  const id = extractField(frontMatter, 'id');
  const term = extractField(frontMatter, 'term');
  const acronym = extractField(frontMatter, 'acronym');
  const categories = extractArrayField(frontMatter, 'categories');
  const relatedTerms = extractArrayField(frontMatter, 'related_terms');

  if (!id || !term) {
    console.warn(`Missing required fields (id/term) in ${filePath}`);
    return null;
  }

  // Extract definition - it's the first paragraph after frontmatter, before any headings
  const definitionMatch = bodyContent.match(/^([^\n#]+(?:\n(?![#\n])[^\n]+)*)/);
  const definition = definitionMatch ? definitionMatch[1].trim().substring(0, 300) : '';

  // Generate relative doc path for linking
  const fileName = path.basename(filePath, '.md');
  const docPath = `/docs/glossary/terms/${fileName}`;

  return {
    id,
    term,
    acronym: acronym || undefined,
    definition,
    categories,
    relatedTerms,
    docPath
  };
}

/**
 * Extract a single field value from YAML frontmatter
 */
function extractField(frontMatter, fieldName) {
  // Split into lines for more precise parsing
  const lines = frontMatter.split('\n');

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    const match = line.match(new RegExp(`^${fieldName}:\\s*(.*)$`));

    if (match) {
      const value = match[1].trim();

      // Empty value or starts with array indicator
      if (value === '' || value.startsWith('-') || value.endsWith(':')) {
        return null;
      }

      return value.replace(/^["']|["']$/g, '');
    }
  }

  return null;
}

/**
 * Extract an array field from YAML frontmatter
 */
function extractArrayField(frontMatter, fieldName) {
  const match = frontMatter.match(new RegExp(`${fieldName}:\\s*\\[([^\\]]+)\\]`));
  if (match) {
    return match[1].split(',').map(item => item.trim().replace(/^["']|["']$/g, ''));
  }

  // Try multiline array format
  const multilineRegex = new RegExp(`${fieldName}:\\s*\\n((?:  - .+\\n?)+)`);
  const multilineMatch = frontMatter.match(multilineRegex);
  if (multilineMatch) {
    return multilineMatch[1]
      .split('\n')
      .filter(line => line.trim().startsWith('- '))
      .map(line => line.trim().substring(2).trim());
  }

  return [];
}

// Run if called directly
if (require.main === module) {
  generateGlossaryIndex()
    .then(count => {
      process.exit(0);
    })
    .catch(error => {
      console.error('Fatal error:', error);
      process.exit(1);
    });
}

module.exports = { generateGlossaryIndex };
