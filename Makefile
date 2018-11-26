all:
	pandoc -o NaviGator_paper.pdf --bibliography=bibliography.bib --csl=bibliography.csl -s -f markdown+raw_tex --template=template.latex NaviGator_paper.md
