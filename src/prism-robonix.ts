import type {PrismTheme} from 'prism-react-renderer';

// The code palette starts from the Robonix mark (#283689). Semantic colors
// remain distinct, but use the same cool indigo/blue family instead of an
// unrelated editor theme.
export const robonixLight: PrismTheme = {
  plain: {
    color: '#20263D',
    backgroundColor: '#F5F6FC',
  },
  styles: [
    {
      types: ['comment', 'prolog', 'doctype', 'cdata'],
      style: {color: '#68708E', fontStyle: 'italic'},
    },
    {
      types: ['namespace'],
      style: {color: '#59617D', opacity: 0.9},
    },
    {
      types: ['string', 'char', 'attr-value', 'regex'],
      style: {color: '#0C7166'},
    },
    {
      types: ['number', 'boolean', 'constant', 'symbol'],
      style: {color: '#176B8F'},
    },
    {
      types: ['keyword', 'selector', 'important', 'atrule'],
      style: {color: '#283689', fontWeight: '600'},
    },
    {
      types: ['function', 'class-name', 'builtin'],
      style: {color: '#6B3F8E'},
    },
    {
      types: ['property', 'tag', 'attr-name', 'variable'],
      style: {color: '#304FA1'},
    },
    {
      types: ['operator', 'entity', 'url'],
      style: {color: '#805300'},
    },
    {
      types: ['punctuation'],
      style: {color: '#46506C'},
    },
    {
      types: ['deleted'],
      style: {color: '#A2324A'},
    },
    {
      types: ['inserted'],
      style: {color: '#0C7166'},
    },
  ],
};

export const robonixDark: PrismTheme = {
  plain: {
    color: '#E8EAFA',
    backgroundColor: '#171A2B',
  },
  styles: [
    {
      types: ['comment', 'prolog', 'doctype', 'cdata'],
      style: {color: '#9AA3C2', fontStyle: 'italic'},
    },
    {
      types: ['namespace'],
      style: {color: '#B3BAD4', opacity: 0.9},
    },
    {
      types: ['string', 'char', 'attr-value', 'regex'],
      style: {color: '#73D4C5'},
    },
    {
      types: ['number', 'boolean', 'constant', 'symbol'],
      style: {color: '#71B7DE'},
    },
    {
      types: ['keyword', 'selector', 'important', 'atrule'],
      style: {color: '#AAB5FF', fontWeight: '600'},
    },
    {
      types: ['function', 'class-name', 'builtin'],
      style: {color: '#D6A9F2'},
    },
    {
      types: ['property', 'tag', 'attr-name', 'variable'],
      style: {color: '#8FB0FF'},
    },
    {
      types: ['operator', 'entity', 'url'],
      style: {color: '#F1C47B'},
    },
    {
      types: ['punctuation'],
      style: {color: '#B9C0DC'},
    },
    {
      types: ['deleted'],
      style: {color: '#FF91A8'},
    },
    {
      types: ['inserted'],
      style: {color: '#73D4C5'},
    },
  ],
};
