cmake_minimum_required(VERSION 3.20)

# BQ25756 driver — tri-mode build (ESP-IDF, Zephyr, plain CMake).
# Bus-agnostic: the HAL exposes I²C register read/write callbacks; the
# platform code wraps whichever I²C master driver it has.

if(IDF_VERSION_MAJOR)
	# ── ESP-IDF Component ────────────────────────────────────────────
	idf_component_register(
		SRCS "src/bq25756.c"
		INCLUDE_DIRS "include"
	)
elseif(ZEPHYR_BASE)
	# ── Zephyr Module ────────────────────────────────────────────────
	zephyr_library()
	zephyr_library_sources(src/bq25756.c)
	zephyr_include_directories(include)
else()
	# ── Standard CMake (STM32, desktop, etc.) ────────────────────────
	if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
		project(bq25756
			LANGUAGES C
			VERSION 0.1.0
			DESCRIPTION "Platform-agnostic driver for the TI BQ25756 1S-28S NVDC battery charger"
		)
	endif()

	add_library(bq25756 STATIC
		src/bq25756.c
	)
	target_include_directories(bq25756 PUBLIC include)
	target_compile_features(bq25756 PUBLIC c_std_11)
	target_compile_options(bq25756 PRIVATE -Wall -Wextra -Wpedantic)
endif()
