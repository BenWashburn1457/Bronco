CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O1 -g
SRC       = test_main.cpp test_config.cpp test_motor.cpp test_steering.cpp test_vehicle.cpp
HDR       = $(wildcard *.h ../bronco_mcu/*.h)

.PHONY: test coverage all clean
all: test coverage

test_all: $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $@

test: test_all
	./test_all

# Every non-COM requirement ID in REQUIREMENTS.md must be tagged by a test.
coverage: test_all
	@grep -oE '^\| [A-Z]+-[0-9]+ ' ../docs/REQUIREMENTS.md | tr -d '| ' | grep -v '^COM-' | sort -u > .required
	@./test_all --reqs | sort -u > .covered
	@missing=$$(comm -23 .required .covered); \
	 if [ -n "$$missing" ]; then echo "Requirements with NO test:"; echo "$$missing"; exit 1; \
	 else echo "coverage: all $$(wc -l < .required) testable requirements have tests"; fi

clean:
	rm -f test_all .required .covered
