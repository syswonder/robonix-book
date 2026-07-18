window.ROBONIX_DOC_BUILD = {"branch": "dev-next", "commit": "47632ee82dc4", "sourceDate": "2026-07-15"};
document.addEventListener("DOMContentLoaded", () => {
    const target = document.getElementById("robonix-doc-revision");
    if (!target) return;
    const info = window.ROBONIX_DOC_BUILD;
    target.textContent = "源码 " + info.commit + " · " + info.sourceDate;
});
