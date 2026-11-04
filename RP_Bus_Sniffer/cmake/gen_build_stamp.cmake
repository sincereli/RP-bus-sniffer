# 由 CMake 在每次构建时执行，把当前时间写进一个极小的 C 文件。
#
# 为什么不用 __DATE__ / __TIME__：这两个宏只在“包含它们的 .c 文件被编译”
# 时展开。改的是 sniffer.c 时 RP_Bus_Sniffer.c 不会重编译，开机打印的就还是
# 上一次的时间，容易误判成“烧录没成功”。改用每次构建都重新生成的独立文件。
#
# 用法：cmake -DOUT=<生成的文件路径> -P gen_build_stamp.cmake

string(TIMESTAMP stamp "%Y-%m-%d %H:%M:%S")
file(WRITE "${OUT}"
     "/* 由 cmake/gen_build_stamp.cmake 自动生成，不要手改 */\n"
     "const char fw_build_stamp[] = \"${stamp}\";\n")
