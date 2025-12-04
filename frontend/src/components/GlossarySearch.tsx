import React, { useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Link from '@docusaurus/Link';

interface GlossaryTerm {
  id: string;
  term: string;
  acronym?: string;
  definition: string;
  categories: string[];
  relatedTerms: string[];
  docPath: string;
}

export function GlossarySearch() {
  const [allTerms, setAllTerms] = useState<GlossaryTerm[]>([]);
  const [filteredTerms, setFilteredTerms] = useState<GlossaryTerm[]>([]);
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedCategory, setSelectedCategory] = useState<string>('All');
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;

  useEffect(() => {
    async function fetchGlossaryIndex() {
      try {
        const response = await fetch(`${baseUrl}glossary-index.json`);
        if (!response.ok) {
          throw new Error(`Failed to fetch glossary index: ${response.statusText}`);
        }
        const terms: GlossaryTerm[] = await response.json();
        setAllTerms(terms);
        setFilteredTerms(terms);
      } catch (err: any) {
        console.error('Error fetching glossary index:', err);
        setError('Failed to load glossary terms. Please try again later.');
      } finally {
        setIsLoading(false);
      }
    }

    fetchGlossaryIndex();
  }, [baseUrl]);

  useEffect(() => {
    const lowerCaseSearchTerm = searchTerm.toLowerCase();
    let results = allTerms;

    // Filter by search term
    if (searchTerm) {
      results = results.filter(
        (term) =>
          term.term.toLowerCase().includes(lowerCaseSearchTerm) ||
          term.definition.toLowerCase().includes(lowerCaseSearchTerm) ||
          (term.acronym && term.acronym.toLowerCase().includes(lowerCaseSearchTerm))
      );
    }

    // Filter by category
    if (selectedCategory !== 'All') {
      results = results.filter((term) => term.categories.includes(selectedCategory));
    }

    setFilteredTerms(results);
  }, [searchTerm, selectedCategory, allTerms]);

  // Get unique categories
  const categories = ['All', ...new Set(allTerms.flatMap((term) => term.categories))].sort();

  if (isLoading) {
    return <div className="alert alert--info">Loading glossary...</div>;
  }

  if (error) {
    return <div className="alert alert--danger">{error}</div>;
  }

  return (
    <div className="glossary-search-container">
      <div className="glossary-controls" style={{ marginBottom: '1rem', display: 'flex', gap: '1rem', flexWrap: 'wrap' }}>
        <input
          type="text"
          placeholder="Search glossary terms..."
          value={searchTerm}
          onChange={(e) => setSearchTerm(e.target.value)}
          className="glossary-search-input"
          style={{ flex: '1 1 300px', padding: '0.5rem', fontSize: '1rem', border: '1px solid var(--ifm-color-emphasis-300)', borderRadius: '4px' }}
        />
        <select
          value={selectedCategory}
          onChange={(e) => setSelectedCategory(e.target.value)}
          className="glossary-category-select"
          style={{ padding: '0.5rem', fontSize: '1rem', border: '1px solid var(--ifm-color-emphasis-300)', borderRadius: '4px' }}
        >
          {categories.map((category) => (
            <option key={category} value={category}>
              {category}
            </option>
          ))}
        </select>
      </div>

      <div className="glossary-stats" style={{ marginBottom: '1rem', color: 'var(--ifm-color-emphasis-600)' }}>
        Showing {filteredTerms.length} of {allTerms.length} terms
      </div>

      <div className="glossary-list">
        {filteredTerms.length === 0 && (
          <div className="alert alert--warning">
            No terms found matching your search criteria.
          </div>
        )}
        {filteredTerms.map((term) => (
          <div key={term.id} className="glossary-item" style={{ marginBottom: '2rem', padding: '1rem', border: '1px solid var(--ifm-color-emphasis-200)', borderRadius: '8px' }}>
            <h3 id={term.id} style={{ marginTop: 0 }}>
              <Link to={term.docPath}>{term.term}</Link>
              {term.acronym && <span style={{ marginLeft: '0.5rem', color: 'var(--ifm-color-emphasis-600)', fontSize: '0.9em' }}>({term.acronym})</span>}
            </h3>
            <p style={{ marginBottom: '0.5rem' }}>{term.definition}</p>
            <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem', fontSize: '0.85rem' }}>
              {term.categories.map((cat) => (
                <span key={cat} className="badge badge--secondary">{cat}</span>
              ))}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}
