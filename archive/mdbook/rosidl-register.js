// ROS .msg / .srv syntax highlighting for the IDL reference.
//
// Registered into highlight.js (which mdBook loads) so ```rosidl fenced
// blocks colour: `#` comments, ROS primitive types, qualified type paths
// (std_msgs/Header …), numbers, and the .srv/.action `---` separator.
//
// We do NOT reuse the legacy `ridl` highlighter (theme/ridl-register.js):
// that targets the deprecated ridl-DSL — it uses `//` comments and treats
// `command` / `request` / `response` as keywords, which would mis-highlight
// ordinary ROS field names.
(function () {
  if (typeof window === "undefined" || !window.hljs) return;
  window.hljs.registerLanguage("rosidl", function (hljs) {
    return {
      name: "ROS IDL",
      aliases: ["rosidl", "rosmsg", "rossrv"],
      keywords: {
        built_in:
          "bool byte char float32 float64 int8 uint8 int16 uint16 int32 uint32 int64 uint64 string wstring",
      },
      contains: [
        hljs.HASH_COMMENT_MODE,
        hljs.QUOTE_STRING_MODE,
        { className: "number", begin: /\b\d+(\.\d+)?\b/ },
        // .srv / .action request/response separator line
        { className: "meta", begin: /^---$/ },
        // qualified type, e.g. std_msgs/Header, pkg/msg/Type, with optional []
        {
          className: "type",
          begin: /\b[a-zA-Z][a-zA-Z0-9_]*(?:\/[a-zA-Z0-9_]+)+(?:\[\d*\])?/,
          relevance: 0,
        },
      ],
    };
  });
})();
