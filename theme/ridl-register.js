;hljs.registerLanguage("ridl",function(hljs){
  var k={keyword:"namespace import query command stream request response input output result version payload",literal:"true false"};
  var typePath=/\b[a-z][a-z0-9_]*\/[a-z0-9_]+\/[a-z][a-z0-9_]*(\[\])?/i;
  var nsPath=/\b[a-z][a-z0-9_]*(\/[a-z0-9_]+)+/;
  var builtin=/\b(float32|float64|int8|int16|int32|int64|uint8|uint16|uint32|uint64|bool|string|byte)\b/;
  return{
    name:"RIDL",
    aliases:["ridl"],
    keywords:k,
    contains:[
      hljs.HASH_COMMENT_MODE,
      hljs.QUOTE_STRING_MODE,
      {className:"number",begin:/\b\d+(\.\d+)?\b/},
      {className:"meta",begin:/@\w+/,contains:[{begin:/\(/,end:/\)/,contains:[hljs.QUOTE_STRING_MODE,{className:"number",begin:/\d+(\.\d+)?/}]}]},
      {className:"type",begin:typePath,relevance:0},
      {className:"built_in",begin:builtin},
      {begin:/\bnamespace\s+/,returnBegin:!0,contains:[
        {className:"keyword",begin:/\bnamespace\b/},
        {className:"symbol",begin:nsPath}
      ]},
      {begin:/\bimport\s+/,returnBegin:!0,contains:[
        {className:"keyword",begin:/\bimport\b/},
        {className:"type",begin:typePath}
      ]},
      {begin:/\b(query|command|stream)\s+/,returnBegin:!0,contains:[
        {className:"keyword",begin:/\b(query|command|stream)\b/},
        {className:"title",begin:/\b(?!query|command|stream\b)[a-z_][a-z0-9_]*\b/}
      ]},
      {begin:/\b(request|response|input|output|result|payload)\s+/,returnBegin:!0,contains:[
        {className:"keyword",begin:/\b(request|response|input|output|result|payload)\b/},
        {className:"attr",begin:/\b[a-z_][a-z0-9_]*\b/}
      ]},
      {className:"keyword",begin:/\bversion\b/},
      {className:"variable",begin:/\b[a-z_][a-z0-9_]*\b/,relevance:0},
      {className:"punctuation",begin:/[{};\[\]]/}
    ]
  };
});
