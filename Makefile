NPM ?= npm
HOST ?= 127.0.0.1
PORT ?= 3000

.PHONY: help install dev typecheck build check serve reference clean

help:
	@printf '%s\n' \
	  'make install    Install the locked Node.js dependencies with npm ci' \
	  'make dev        Start the editable local preview (HOST/PORT can be overridden)' \
	  'make typecheck  Check the Docusaurus TypeScript configuration' \
	  'make build      Type-check and create the production site in build/' \
	  'make check      Run every local pre-commit check' \
	  'make serve      Build and serve the production site locally' \
	  'make reference ROBONIX_SOURCE=/path/to/robonix  Regenerate pinned contract and IDL pages' \
	  'make clean      Remove Docusaurus caches and generated site files'

install:
	$(NPM) ci

dev:
	$(NPM) run start -- --host $(HOST) --port $(PORT)

typecheck:
	$(NPM) run typecheck

build: typecheck
	$(NPM) run build

check: build
	python3 scripts/check-built-links.py build
	python3 scripts/check-legacy-urls.py build
	test -s build/search-index.json

serve: build
	$(NPM) run serve -- --host $(HOST) --port $(PORT)

reference:
	@test -n "$(ROBONIX_SOURCE)" || { echo 'ROBONIX_SOURCE is required, for example: make reference ROBONIX_SOURCE=/path/to/robonix'; exit 2; }
	@test -f "$(ROBONIX_SOURCE)/Cargo.toml" || { echo 'ROBONIX_SOURCE must point to a Robonix source checkout'; exit 2; }
	@set -eu; \
	  expected=$$(tr -d '[:space:]' < ROBONIX_SOURCE_REVISION); \
	  actual=$$(git -C "$(ROBONIX_SOURCE)" rev-parse HEAD); \
	  test "$$actual" = "$$expected" || { echo "Robonix source is at $$actual, expected $$expected"; exit 2; }; \
	  test -z "$$(git -C "$(ROBONIX_SOURCE)" status --porcelain)" || { echo 'ROBONIX_SOURCE must be a clean checkout'; exit 2; }; \
	  CARGO_TARGET_DIR="$(ROBONIX_SOURCE)/target" cargo build --manifest-path "$(ROBONIX_SOURCE)/Cargo.toml" --locked --release -p robonix-cli; \
	  scratch=$$(mktemp -d); \
	  trap 'rm -rf "$$scratch"' EXIT HUP INT TERM; \
	  export ROBONIX_HOME="$$scratch/robonix-home"; \
	  "$(ROBONIX_SOURCE)/target/release/rbnx" setup "$(ROBONIX_SOURCE)"; \
	  generated="$$scratch/generated-reference"; \
	  mkdir -p "$$generated"; \
	  "$(ROBONIX_SOURCE)/target/release/rbnx" docs --out-dir "$$generated"; \
	  python3 scripts/normalize-reference.py "$$generated/contracts.md" "$$generated/idl.md"; \
	  install -m 0644 "$$generated/contracts.md" docs/reference/contracts.md; \
	  install -m 0644 "$$generated/idl.md" docs/reference/idl.md

clean:
	$(NPM) run clear
