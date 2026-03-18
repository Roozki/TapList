# Makefile for common libraries

BUILD_DIR:=build

all: clean configure build run

configure:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake ..

build: configure 
	@$(MAKE) -C $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

run:
	@$(BUILD_DIR)/server

.PHONY: all configure build clean run