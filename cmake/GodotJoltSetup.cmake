include_guard()

include(GodotJoltConfigs)
include(GodotJoltOptions)

# Set EP_BASE so we get a nicer directory structure for external projects
set_directory_properties(PROPERTIES EP_BASE ${PROJECT_BINARY_DIR}/External)

if(PROJECT_IS_TOP_LEVEL)
	# Group targets into folders for any IDE that supports it (like Visual Studio)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)

	# Ensure we output to the `bin` directory when building from Visual Studio
	set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD TRUE)
endif()

if(NOT DEFINED GDJOLT_TARGET_ARCHITECTURES)
	if(CMAKE_HOST_SYSTEM_NAME STREQUAL Darwin)
		set(GDJOLT_TARGET_ARCHITECTURES ${CMAKE_OSX_ARCHITECTURES})
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES [[^(i386|i686|x86)$]])
		set(GDJOLT_TARGET_ARCHITECTURES x86)
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES [[^(AMD64|amd64|x86_64)$]])
		set(GDJOLT_TARGET_ARCHITECTURES x64)
	else()
		message(FATAL_ERROR "Unhandled system processor: '${CMAKE_SYSTEM_PROCESSOR}'.")
	endif()
endif()

if(DEFINED ENV{VSCMD_ARG_TGT_ARCH})
	if(NOT $ENV{VSCMD_ARG_TGT_ARCH} STREQUAL ${GDJOLT_TARGET_ARCHITECTURES})
		message(FATAL_ERROR
			"Mismatching target architectures. The current Visual Studio environment is set up for "
			"the '$ENV{VSCMD_ARG_TGT_ARCH}' architecture, but we intend to build for the "
			"'${GDJOLT_TARGET_ARCHITECTURES}' architecture.\nPlease make sure you match the "
			"developer command prompt with the intended target architecture.\n"
			"You should delete any build folder generated by this command before trying again."
		)
	endif()
endif()

macro(is_targeting instruction_sets output_variable)
	if(GDJOLT_X86_INSTRUCTION_SET MATCHES "^(${instruction_sets})$")
		set(${output_variable} TRUE)
	else()
		set(${output_variable} FALSE)
	endif()
endmacro()

is_targeting(AVX512 GDJOLT_USE_AVX512)
is_targeting(AVX512|AVX2 GDJOLT_USE_AVX2)
is_targeting(AVX512|AVX2 GDJOLT_USE_BMI1)
is_targeting(AVX512|AVX2 GDJOLT_USE_FMA3)
is_targeting(AVX512|AVX2 GDJOLT_USE_F16C)
is_targeting(AVX512|AVX2|AVX GDJOLT_USE_AVX)
is_targeting(AVX512|AVX2|AVX GDJOLT_USE_SSE4_2)
is_targeting(AVX512|AVX2|AVX|SSE2 GDJOLT_USE_SSE2)

if(GDJOLT_CROSS_PLATFORM_DETERMINISTIC)
	set(GDJOLT_USE_FMA3 FALSE)
endif()
