from pathlib import Path

vw = Path("vision_worker.c").read_text(encoding="utf-8")
old = """        } else if (fid < last_fid) {
        need_reseed = 1;
    }"""
new = """        } else if (fid < last_fid) {
        last_seen_seq = 0;
    }"""
if old in vw:
    vw = vw.replace(old, new, 1)
    Path("vision_worker.c").write_text(vw, encoding="utf-8", newline="\n")
    print("Updated vision_worker.c loop handling")
else:
    print("Check vision_worker.c manually")