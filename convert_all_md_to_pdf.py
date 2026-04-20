import os
import pypandoc
from pathlib import Path

def convert_md_to_pdf(root_dir):
    md_files = []
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith('.md'):
                md_files.append(os.path.join(root, file))
    
    for md_file in md_files:
        pdf_file = md_file.replace('.md', '.pdf')
        print(f"Converting {md_file} to {pdf_file}...")
        try:
            # Add extra arguments for PDF conversion if needed, e.g., geometry or margins
            # extra_args = ['-V', 'geometry:margin=1in']
            pypandoc.convert_file(md_file, 'pdf', outputfile=pdf_file)
            print(f"Successfully converted {md_file}")
        except Exception as e:
            print(f"Error converting {md_file}: {e}")

if __name__ == "__main__":
    parts_path = Path("src/case_studies/L_rodmass/parts")
    if parts_path.exists():
        convert_md_to_pdf(str(parts_path))
    else:
        print(f"Directory {parts_path} not found.")
