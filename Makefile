NPM ?= npm
PYTHON ?= python3
API_PYTHON ?= $(PYTHON)
API_CARGO_TARGET_DIR ?= $(ROBONIX_SOURCE)/target
HOST ?= 127.0.0.1
PORT ?= 3000
SPHINX_MOCKS ?= torch,rclpy,cv2,open3d,scipy,sklearn

.PHONY: help install dev typecheck codeblocks page-evidence validation-artifacts assets build check serve source-check api-install api-rust api-python api-check api full-build full-check full-serve reference clean

help:
	@printf '%s\n' \
	  'make install    Install the locked Node.js dependencies with npm ci' \
	  'make dev        Start the editable local preview (HOST/PORT can be overridden)' \
	  'make typecheck  Check the Docusaurus TypeScript configuration' \
	  'make codeblocks Check fenced Bash, JavaScript, JSON, Python, TOML, and YAML syntax' \
	  'make page-evidence Check page provenance, fenced-block classifications, and execution records' \
	  'make validation-artifacts Check browser and source-deck audit reports' \
	  'make assets     Check documentation image paths and alternative text' \
	  'make build      Type-check and create the production site in build/' \
	  'make check      Run every local pre-commit check' \
	  'make serve      Build and serve the production site locally' \
	  'make api-install ROBONIX_SOURCE=/path/to/robonix  Install Python API-doc dependencies in .venv-api' \
	  'make full-check ROBONIX_SOURCE=/path/to/robonix API_PYTHON=.venv-api/bin/python  Build and verify the handbook plus Rust/Python API docs' \
	  'make full-serve ROBONIX_SOURCE=/path/to/robonix API_PYTHON=.venv-api/bin/python  Serve the complete production site' \
	  'make reference ROBONIX_SOURCE=/path/to/robonix  Regenerate pinned contract and IDL pages' \
	  'make clean      Remove Docusaurus caches and generated site files'

install:
	$(NPM) ci

dev:
	$(NPM) run start -- --host $(HOST) --port $(PORT)

typecheck:
	$(NPM) run typecheck

codeblocks:
	$(NPM) run check:code-blocks

page-evidence:
	$(NPM) run check:page-evidence

validation-artifacts:
	$(NPM) run check:validation-artifacts

assets:
	$(NPM) run check:assets

build: typecheck
	$(NPM) run build

check: codeblocks page-evidence validation-artifacts assets build
	python3 scripts/check-built-links.py build
	python3 scripts/check-legacy-urls.py build
	test -s build/search-index.json

serve: build
	$(NPM) run serve -- --host $(HOST) --port $(PORT)

source-check:
	@test -n "$(ROBONIX_SOURCE)" || { echo 'ROBONIX_SOURCE is required, for example: ROBONIX_SOURCE=/path/to/robonix'; exit 2; }
	@test -f "$(ROBONIX_SOURCE)/Cargo.toml" || { echo 'ROBONIX_SOURCE must point to a Robonix source checkout'; exit 2; }
	@set -eu; \
	  expected=$$(tr -d '[:space:]' < ROBONIX_SOURCE_REVISION); \
	  actual=$$(git -C "$(ROBONIX_SOURCE)" rev-parse HEAD); \
	  test "$$actual" = "$$expected" || { echo "Robonix source is at $$actual, expected $$expected"; exit 2; }; \
	  test -z "$$(git -C "$(ROBONIX_SOURCE)" status --porcelain)" || { echo 'ROBONIX_SOURCE must be a clean checkout'; exit 2; }

api-install: source-check
	$(PYTHON) -m venv .venv-api
	.venv-api/bin/python -m pip install --upgrade pip
	.venv-api/bin/python -m pip install sphinx furo numpy grpcio grpcio-tools protobuf pyyaml
	.venv-api/bin/python -m pip install "$(ROBONIX_SOURCE)/pylib/robonix-api"

api-rust: source-check
	CARGO_TARGET_DIR="$(API_CARGO_TARGET_DIR)" cargo doc --manifest-path "$(ROBONIX_SOURCE)/Cargo.toml" --locked --no-deps --workspace
	rm -rf build/api/rust
	mkdir -p build/api/rust
	cp -R "$(API_CARGO_TARGET_DIR)/doc/." build/api/rust/

api-python: source-check
	@set -eu; \
	  rbnx="$(API_CARGO_TARGET_DIR)/release/rbnx"; \
	  if test ! -x "$$rbnx"; then \
	    CARGO_TARGET_DIR="$(API_CARGO_TARGET_DIR)" cargo build --manifest-path "$(ROBONIX_SOURCE)/Cargo.toml" --locked --release -p robonix-cli; \
	  fi; \
	  scratch=$$(mktemp -d); \
	  trap 'rm -rf "$$scratch"' EXIT HUP INT TERM; \
	  export CARGO_TARGET_DIR="$(API_CARGO_TARGET_DIR)"; \
	  export ROBONIX_HOME="$$scratch/robonix-home"; \
	  "$$rbnx" setup "$(ROBONIX_SOURCE)" >/dev/null; \
	  mkdir -p "$$scratch/scene"; \
	  cp "$(ROBONIX_SOURCE)/system/scene/package_manifest.yaml" "$$scratch/scene/package_manifest.yaml"; \
	  api_python_dir=$$(cd "$$(dirname "$(API_PYTHON)")" && pwd); \
	  PATH="$$api_python_dir:$$PATH" "$$rbnx" codegen -p "$$scratch/scene" --mcp --out-dir "$$scratch/codegen"; \
	  rm -rf build/api/python; \
	  mkdir -p build/api/python; \
	  ROBONIX_SPHINX_MOCKS="$(SPHINX_MOCKS)" \
	  ROBONIX_API_SRC="$(ROBONIX_SOURCE)/pylib/robonix-api" \
	  ROBONIX_SCENE_SRC="$(ROBONIX_SOURCE)/system/scene" \
	  ROBONIX_SCENE_CODEGEN="$$scratch/codegen" \
	    $(API_PYTHON) -m sphinx -b html apidoc/python build/api/python

api-check:
	test -s build/api/rust/robonix_atlas/index.html
	test -s build/api/rust/robonix_executor/index.html
	test -s build/api/rust/robonix_pilot/index.html
	test -s build/api/rust/robonix_liaison/index.html
	test -s build/api/rust/robonix_codegen/index.html
	test -s build/api/rust/rbnx/index.html
	test -s build/api/python/_autosummary/robonix_api.html
	test -s build/api/python/_autosummary/scene_service.html
	test -s build/api/python/public-api.html
	grep -q 'robonix_api.Primitive' build/api/python/_autosummary/robonix_api.html
	grep -q 'robonix_api.Service' build/api/python/_autosummary/robonix_api.html
	grep -q 'robonix_api.Skill' build/api/python/_autosummary/robonix_api.html
	grep -q 'robonix_api.Primitive' build/api/python/public-api.html
	grep -q 'robonix_api.atlas._Atlas.find_capability' build/api/python/public-api.html
	grep -q 'robonix_api.atlas._Atlas.find_unique_capability' build/api/python/public-api.html
	grep -q 'connect_capability' build/api/python/public-api.html
	grep -q 'create_publisher' build/api/python/public-api.html
	grep -q 'create_subscription' build/api/python/public-api.html

api: source-check
	$(MAKE) api-rust ROBONIX_SOURCE="$(ROBONIX_SOURCE)" API_CARGO_TARGET_DIR="$(API_CARGO_TARGET_DIR)"
	$(MAKE) api-python ROBONIX_SOURCE="$(ROBONIX_SOURCE)" API_PYTHON="$(API_PYTHON)" API_CARGO_TARGET_DIR="$(API_CARGO_TARGET_DIR)"
	$(MAKE) api-check

full-build: build
	$(MAKE) api ROBONIX_SOURCE="$(ROBONIX_SOURCE)" API_PYTHON="$(API_PYTHON)" API_CARGO_TARGET_DIR="$(API_CARGO_TARGET_DIR)"

full-check: codeblocks page-evidence assets full-build
	python3 scripts/check-built-links.py build
	python3 scripts/check-legacy-urls.py build
	test -s build/search-index.json
	$(MAKE) api-check

full-serve: full-build
	$(NPM) run serve -- --host $(HOST) --port $(PORT)

reference: source-check
	@set -eu; \
	  CARGO_TARGET_DIR="$(API_CARGO_TARGET_DIR)" cargo build --manifest-path "$(ROBONIX_SOURCE)/Cargo.toml" --locked --release -p robonix-cli; \
	  scratch=$$(mktemp -d); \
	  trap 'rm -rf "$$scratch"' EXIT HUP INT TERM; \
	  export CARGO_TARGET_DIR="$(API_CARGO_TARGET_DIR)"; \
	  export ROBONIX_HOME="$$scratch/robonix-home"; \
	  "$(API_CARGO_TARGET_DIR)/release/rbnx" setup "$(ROBONIX_SOURCE)"; \
	  generated="$$scratch/generated-reference"; \
	  mkdir -p "$$generated"; \
	  "$(API_CARGO_TARGET_DIR)/release/rbnx" docs --out-dir "$$generated"; \
	  python3 scripts/normalize-reference.py "$$generated/contracts.md" "$$generated/idl.md"; \
	  install -m 0644 "$$generated/contracts.md" docs/reference/contracts.md; \
	  install -m 0644 "$$generated/idl.md" docs/reference/idl.md

clean:
	$(NPM) run clear
