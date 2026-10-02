import { TRANSLATION_API_URL } from '../config/api';

// Text blocks translated one-to-one, so headings, lists and tables keep their structure.
const BLOCK_SELECTOR = 'h1, h2, h3, h4, h5, h6, p, li, th, td, blockquote, figcaption, summary';
// Keeps each request well under the backend's 4000-token output limit.
const MAX_BATCH_CHARS = 1500;
const CONCURRENCY = 4;

// Urdu/Arabic-Indic digits -> ASCII, in case the model localizes the [n] markers.
const toAsciiDigits = (s: string) =>
  s.replace(/[۰-۹]/g, d => String(d.charCodeAt(0) - 0x06f0))
   .replace(/[٠-٩]/g, d => String(d.charCodeAt(0) - 0x0660));

function getBlocks(root: HTMLElement): HTMLElement[] {
  return Array.from(root.querySelectorAll<HTMLElement>(BLOCK_SELECTOR)).filter(el =>
    !el.querySelector(BLOCK_SELECTOR) &&   // innermost blocks only
    !el.closest('pre') &&                  // never translate code
    /[A-Za-z]/.test(el.textContent || '')
  );
}

async function translateBatch(chapterId: string, texts: string[]): Promise<Map<number, string>> {
  const content = texts.map((t, i) => `[${i + 1}] ${t}`).join('\n');
  const response = await fetch(`${TRANSLATION_API_URL}/translate`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ chapter_id: chapterId, content }),
  });
  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new Error(errorData.detail || response.statusText);
  }
  const data = await response.json();
  const result = new Map<number, string>();
  for (const line of String(data.translated_content || '').split('\n')) {
    const m = toAsciiDigits(line).match(/^\s*\[(\d+)\]\s*(.+)$/);
    if (m) result.set(Number(m[1]) - 1, m[2].trim());
  }
  return result;
}

/**
 * Translates a copy of `root` to Urdu block by block and returns its HTML.
 * Translations are inserted as text only, never as HTML. Blocks whose
 * translation is missing stay in English. Throws if nothing was translated.
 */
export async function translateElementToUrdu(root: HTMLElement, chapterId: string): Promise<string> {
  const copy = root.cloneNode(true) as HTMLElement;
  const blocks = getBlocks(copy);

  const batches: HTMLElement[][] = [];
  let current: HTMLElement[] = [];
  let size = 0;
  for (const block of blocks) {
    const len = (block.textContent || '').length;
    if (current.length && size + len > MAX_BATCH_CHARS) {
      batches.push(current);
      current = [];
      size = 0;
    }
    current.push(block);
    size += len;
  }
  if (current.length) batches.push(current);

  let translated = 0;
  let lastError: unknown = null;
  let next = 0;
  const worker = async () => {
    while (next < batches.length) {
      const batch = batches[next++];
      try {
        const texts = batch.map(el => (el.textContent || '').replace(/\s+/g, ' ').trim());
        const result = await translateBatch(chapterId, texts);
        batch.forEach((el, i) => {
          const urdu = result.get(i);
          if (urdu) {
            el.textContent = urdu;
            translated++;
          }
        });
      } catch (error) {
        lastError = error;
      }
    }
  };
  await Promise.all(Array.from({ length: Math.min(CONCURRENCY, batches.length) }, worker));

  if (translated === 0) {
    throw lastError instanceof Error ? lastError : new Error('No translated content received');
  }
  return copy.innerHTML;
}
