import os
import sys
import requests

working_path = sys.argv[1]

# Header
ui_html_h = """
#ifndef _UI_HTML_H_
#define _UI_HTML_H_

#include <unistd.h>

extern const char ui_index_main[];
extern const ssize_t ui_index_main_len;

extern const char ui_jquery_sparkline_js_gz[];
extern const ssize_t ui_jquery_sparkline_js_gz_len;

#endif // _UI_HTML_H_
"""
with open(os.path.join(working_path, "./ui_html.h"), "w") as f:
    f.write(ui_html_h)

# Actual data

def format_hex(data, basename):
    ret = ""
    BYTES_PER_LINE=15
    ret += f'const char {basename}[] = {{\n'
    (whole_lines, final_line_size) = divmod(len(data), BYTES_PER_LINE)
    for i in range(whole_lines):
        if final_line_size == 0 and i == whole_lines - 1:
            endchar = '\n};'
        else:
            endchar=','
        ret += (' ' * 4) + ', '.join([f"0x{b:02x}" for b in data[i*BYTES_PER_LINE:(i*BYTES_PER_LINE)+BYTES_PER_LINE]]) + f'{endchar}\n'
    if final_line_size > 0:
        ret += (' ' * 4) + ', '.join([f"0x{b:02x}" for b in data[whole_lines*BYTES_PER_LINE:]]) + '\n};\n'
    ret += f"const ssize_t {basename}_len = {len(data)};\n\n"

    return ret

ui_html_c = '#include <unistd.h>\n\n'
ui_html_c += '#include "ui_html.h"\n\n'

# First the index page
with open(os.path.join(working_path, "index.html"), 'rb') as f:
    d = f.read()
ui_html_c += format_hex(d, "ui_index_main")

# Pull in jquery-sparklines minified gz
r = requests.get("https://omnipotent.net/jquery.sparkline/2.1.2/jquery.sparkline.min.js.gz")
ui_html_c += format_hex(r.content, "ui_jquery_sparkline_js_gz")

with open(os.path.join(working_path, "./ui_html.c"), "w") as f:
    f.write(ui_html_c)