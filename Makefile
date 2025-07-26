.PHONY: all debug release

debug: 
	conan install . --output-folder=build/debug --build=missing -s build_type=Debug
	cd build/debug ; cmake ../..  -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=$(HOME)/.local
	cmake --build build/debug --parallel

debug-install: debug
	cmake --build build/debug --parallel --target install

release:
	conan install . --output-folder=build/release --build=missing -s build_type=Release
	cd build/release ; cmake ../..  -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(HOME)/.local
	cmake --build build/release --parallel

release-install: release
	cmake --build build/release --parallel --target install

all: debug release

clean:
	rm -rf build
	
