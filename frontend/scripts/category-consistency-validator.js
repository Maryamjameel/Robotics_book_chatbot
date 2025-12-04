const fs = require('fs');
const path = require('path');
const { glob } = require('glob');

async function validateCategoryConsistency() {
  console.log('Starting category consistency validation...');

  const glossaryTermsPath = path.join(__dirname, '..', 'docs', 'glossary', 'terms');
  const categoryFilePath = path.join(__dirname, '..', 'docs', 'glossary', '_category_.json');
  const markdownFiles = await glob('**/*.md', { cwd: glossaryTermsPath });

  const categoryFileContent = fs.readFileSync(categoryFilePath, 'utf8');
  const categoryData = JSON.parse(categoryFileContent);
  const approvedCategories = new Set(categoryData.customProps?.approvedCategories || []);

  if (approvedCategories.size === 0) {
    console.error('ERROR: No approved categories found in _category_.json. Please define them under customProps.approvedCategories.');
    process.exit(1);
  }

  let errorsFound = false;

  for (const file of markdownFiles) {
    const filePath = path.join(glossaryTermsPath, file);
    const content = fs.readFileSync(filePath, 'utf8');
    const termId = content.match(/^id: "([a-zA-Z0-9-]+)"/m)?.[1] || path.basename(file, '.md');

    const categoriesMatch = content.match(/categories:\s*
([\s\S]*?)(?:\n[a-zA-Z]|$)/);

    if (categoriesMatch && categoriesMatch[1]) {
      const categories = categoriesMatch[1].split('\n').map(line => line.trim().replace(/^- "|"/g, '')).filter(Boolean);

      if (categories.length === 0 || categories.length > 3) {
        console.error(`ERROR: Term '${termId}' in '${file}' has ${categories.length} categories. Expected 1 to 3 categories.`);
        errorsFound = true;
      }

      for (const category of categories) {
        if (!approvedCategories.has(category)) {
          console.error(`ERROR: Term '${termId}' in '${file}' uses unapproved category: '${category}'. Approved categories are: ${Array.from(approvedCategories).join(', ')}`);
          errorsFound = true;
        }
      }
    } else {
      console.error(`ERROR: Term '${termId}' in '${file}' has no categories defined in its frontmatter.`);
      errorsFound = true;
    }
  }

  if (!errorsFound) {
    console.log('Category consistency validation successful: All terms use approved categories within the 1-3 limit.');
  } else {
    console.error('Category consistency validation failed: Category issues detected.');
    process.exit(1);
  }
}

validateCategoryConsistency();
