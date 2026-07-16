import {Prism} from 'prism-react-renderer';

const primitiveTypes = [
  'bool',
  'byte',
  'char',
  'float32',
  'float64',
  'int8',
  'int16',
  'int32',
  'int64',
  'string',
  'uint8',
  'uint16',
  'uint32',
  'uint64',
  'wstring',
].join('|');

Prism.languages.rosidl = {
  comment: {
    pattern: /#.*/,
    greedy: true,
  },
  separator: {
    pattern: /^---$/m,
    alias: 'operator',
  },
  type: {
    pattern: new RegExp(
      `\\b(?:${primitiveTypes}|[A-Za-z][\\w]*(?:\\/[A-Za-z][\\w]*)?)(?:\\[(?:<=?\\d+)?\\])?(?=\\s+[A-Za-z_][\\w]*)`,
    ),
    alias: 'class-name',
  },
  constant: {
    pattern: /\b[A-Z][A-Z0-9_]*(?=\s*=)/,
    alias: 'constant',
  },
  property: /\b[a-zA-Z_][\w]*(?=\s*(?:=|#|$))/m,
  boolean: /\b(?:true|false)\b/,
  number: /(?:\b0x[\da-f]+|(?:\B[-+]|\b)\d+(?:\.\d+)?(?:e[-+]?\d+)?)\b/i,
  string: {
    pattern: /"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'/,
    greedy: true,
  },
  operator: /=/,
  punctuation: /[\[\](),<>]/,
};

Prism.languages.msg = Prism.languages.rosidl;
Prism.languages.srv = Prism.languages.rosidl;
