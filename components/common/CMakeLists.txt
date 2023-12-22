set(srcs safe_printf.cpp structures.cpp crc.cpp)

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS . )

