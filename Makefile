paper:
	pandoc -f markdown+raw_tex --variable classoption=twocolumn --variable papersize=a4paper -s NaviGator_paper.md -o Navigator_paper.pdf

paper-bib:
	pandoc -f markdown+raw_tex --filter pandoc-citeproc --bibliography=paper.bib --variable classoption=twocolumn --variable papersize=a4paper -s NaviGator_paper.md -o Navigator_paper.pdf
