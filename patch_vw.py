from pathlib import Path

p = Path("vision_worker.c")
text = p.read_text(encoding="utf-8")
target = "else if (fid < last_fid)"
idx = text.find(target)
if idx == -1:
    print("Line not found")
else:
    brace_start = text.find("{", idx)
    brace_end = text.find("}", brace_start)
    if brace_start != -1 and brace_end != -1:
        patched = text[:brace_start + 1] + "\n            last_seen_seq = 0;\n        " + text[brace_end:]
        p.write_text(patched, encoding="utf-8", newline="\n")
        print("Patched fid < last_fid in vision_worker.c")
    else:
        print("Braces not found")