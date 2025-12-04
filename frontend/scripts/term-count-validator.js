const fs = require('fs');
const path = require('path');
const { glob } = require('glob');

async function validateTermCount() {
  console.log('Starting glossary term count validation...');

  const glossaryTermsPath = path.join(__dirname, '..', 'docs', 'glossary', 'terms');
  const markdownFiles = await glob('**/*.md', { cwd: glossaryTermsPath });

  const termCount = markdownFiles.length;
  const minTerms = 60;
  const maxTerms = 80;

  if (termCount >= minTerms && termCount <= maxTerms) {
    console.log(`Term count validation successful: ${termCount} terms found (within ${minTerms}-${maxTerms} range).`);
  } else {
    console.error(`ERROR: Term count validation failed. Found ${termCount} terms, but expected between ${minTerms} and ${maxTerms}.`);
    process.exit(1);
  }

  console.log('Found terms:');
  markdownFiles.forEach(file => console.log(`- ${file}`));
}

validateTermCount();
