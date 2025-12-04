
const fs = require('fs');
const path = require('path');
const { glob } = require('glob');

async function validateLinkIntegrity() {
  console.log('Starting link integrity validation...');

  const glossaryTermsPath = path.join(__dirname, '..', 'docs', 'glossary', 'terms');
  const markdownFiles = await glob('**/*.md', { cwd: glossaryTermsPath });

  const allGlossaryIds = new Set();
  const allGlossaryFilePaths = new Set();

  markdownFiles.forEach(file => {
    const filePath = path.join(glossaryTermsPath, file);
    const content = fs.readFileSync(filePath, 'utf8');
    const match = content.match(/^id: "([a-zA-Z0-9-]+)"/m);
    if (match && match[1]) {
      allGlossaryIds.add(match[1]);
      allGlossaryFilePaths.add(`/docs/glossary/terms/${path.basename(file, '.md')}`);
    }
  });

  let errorsFound = false;

  for (const file of markdownFiles) {
    const filePath = path.join(glossaryTermsPath, file);
    const content = fs.readFileSync(filePath, 'utf8');
    const termId = content.match(/^id: "([a-zA-Z0-9-]+)"/m)?.[1] || path.basename(file, '.md');

    const markdownLinkRegex = /\b(?:https?|ftp):\/\/[^\s/$.?#].[^\s]*|(?:\[[^\]]+\]\((?!https?:\/\/)([^)]*?)\))/g;
    let match;

    while ((match = markdownLinkRegex.exec(content)) !== null) {
      const link = match[1];

      if (link && link.startsWith('/docs/glossary/terms/')) {
        const targetId = link.split('/').pop();
        if (!allGlossaryIds.has(targetId)) {
          console.error(`ERROR: Term '${termId}' in '${file}' links to non-existent glossary term: '${link}'`);
          errorsFound = true;
        }
      } else if (link) {
        // For now, only log external links without full validation
        // console.log(`INFO: External link found in '${file}': '${link}'`);
      }
    }

    // Validate related_terms in YAML frontmatter
    const relatedTermsMatch = content.match(/related_terms:\s*
(\s*-\s*\"([a-zA-Z0-9-]+)\")*/);

    if (relatedTermsMatch) {
      const relatedTerms = relatedTermsMatch[0].split('\n').slice(1).map(line => line.trim().replace(/^- "|"/g, ''));
      for (const relatedTerm of relatedTerms) {
        if (!allGlossaryIds.has(relatedTerm) && relatedTerm !== '') {
          console.error(`ERROR: Term '${termId}' in '${file}' lists non-existent related term in frontmatter: '${relatedTerm}'`);
          errorsFound = true;
        }
      }
    }
  }

  if (!errorsFound) {
    console.log('Link integrity validation successful: No broken internal glossary links found.');
  } else {
    console.error('Link integrity validation failed: Broken internal glossary links detected.');
    process.exit(1);
  }
}

validateLinkIntegrity();
