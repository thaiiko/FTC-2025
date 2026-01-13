import os
import subprocess

def main():
    if not os.path.exists("equations.tex"):
        print("Error: equations.tex not found")
        return

    with open("equations.tex", "r") as f:
        content = f.read()

    doc_start_marker = r"\begin{document}"
    
    # Extract preamble
    try:
        start_doc = content.index(doc_start_marker)
        preamble = content[:start_doc]
        lines = preamble.splitlines()
        preamble_lines = [l for l in lines if not l.strip().startswith(r"\documentclass")]
        preamble = "\n".join(preamble_lines)
    except ValueError:
        print(f"Error: {doc_start_marker} not found")
        return

    # Environments to extract
    envs = [
        (r"\begin{equation}", r"\end{equation}"),
        (r"\begin{align}", r"\end{align}"),
        (r"\begin{align*}\n", r"\end{align*}\n"),
        (r"[", r"]")
    ]

    found_eqs = []
    search_pos = start_doc
    while search_pos < len(content):
        earliest_start = -1
        selected_env = None
        for start_tag, end_tag in envs:
            pos = content.find(start_tag, search_pos)
            if pos != -1 and (earliest_start == -1 or pos < earliest_start):
                earliest_start = pos
                selected_env = (start_tag, end_tag)
        if earliest_start == -1:
            break
        start_tag, end_tag = selected_env
        end_pos = content.find(end_tag, earliest_start + len(start_tag))
        if end_pos == -1:
            search_pos = earliest_start + len(start_tag)
            continue
        equation = content[earliest_start : end_pos + len(end_tag)]
        found_eqs.append(equation)
        search_pos = end_pos + len(end_tag)

    print(f"Extracted {len(found_eqs)} equations.")

    if not os.path.exists("equations_svg"):
        os.makedirs("equations_svg")

    for i, eq in enumerate(found_eqs, 1):
        tex_file = f"equations_svg/eq_{i}.tex"
        dvi_file = f"equations_svg/eq_{i}.dvi"
        svg_file = f"equations_svg/eq_{i}.svg"
        
        with open(tex_file, "w") as f:
            f.write(r"\documentclass{standalone}" + "\n")
            f.write(preamble + "\n")
            # Ensure amsmath is included if not in preamble
            if "amsmath" not in preamble:
                f.write(r"\usepackage{amsmath}" + "\n")
            f.write(r"\begin{document}" + "\n")
            f.write(eq + "\n")
            f.write(r"\end{document}" + "\n")
        
        try:
            # Run latex, ignore non-zero exit if DVI produced
            subprocess.run(["latex", "-interaction=nonstopmode", "-output-directory=equations_svg", tex_file], 
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            if os.path.exists(dvi_file):
                # Run dvisvgm
                subprocess.run(["dvisvgm", "--no-fonts", "-o", svg_file, dvi_file], 
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if os.path.exists(svg_file):
                    print(f"Generated eq_{i}.svg")
                else:
                    print(f"Failed to create SVG for eq_{i} despite DVI existence.")
            else:
                print(f"Failed to create DVI for eq_{i}. Attempting pdflatex + pdf2svg (if available) or checking log.")
                # We don't have pdf2svg, so let's check why latex failed
                log_file = f"equations_svg/eq_{i}.log"
                if os.path.exists(log_file):
                    with open(log_file, "r") as lf:
                        log_content = lf.read()
                        if "Undefined control sequence" in log_content:
                            print(f"Eq {i} error: Undefined control sequence.")
                        else:
                            print(f"Eq {i} log: {log_content[-200:].strip()}")
        except Exception as e:
            print(f"Error processing eq_{i}: {e}")
        finally:
            base = f"equations_svg/eq_{i}"
            for ext in [".tex", ".dvi", ".aux", ".log"]:
                path = base + ext
                if os.path.exists(path):
                    os.remove(path)

if __name__ == "__main__":
    main()
