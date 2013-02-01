reportBioMAV.pdf: reportBioMAV.tex
	pdflatex  reportBioMAV
	bibtex reportBioMAV
	pdflatex reportBioMAV
	pdflatex reportBioMAV

test:
	open reportBioMAV.pdf

clean:
	rm *.bbl *.dvi *.aux *.log *.blg *.pdf

edit:
	vim reportBioMAV.tex

