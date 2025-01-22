all: output/proposal.pdf

.PHONY: all clean

output/%.pdf: %.tex refs.bib
	latex --output-dir output "$<"
	biber --output-dir output "$*"
	pdflatex --output-dir output "$<"

clean:
	git clean -fX output
