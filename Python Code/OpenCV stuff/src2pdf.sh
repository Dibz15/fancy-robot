#!/usr/bin/bash

tex_file=tmptex ## Random temp file name

cat<<EOF >$tex_file   ## Print the tex file header
\documentclass{article}
\usepackage{listings}
\usepackage[utf8]{inputenc}
\usepackage[usenames,dvipsnames]{color}  %% Allow color names

% Default fixed font does not support bold face
\DeclareFixedFont{\ttb}{T1}{txtt}{bx}{n}{12} % for bold
\DeclareFixedFont{\ttm}{T1}{txtt}{m}{n}{12}  % for normal

% Custom colors
\usepackage{color}
\definecolor{deepblue}{rgb}{0,0,0.5}
\definecolor{deepred}{rgb}{0.6,0,0}
\definecolor{deepgreen}{rgb}{0,0.5,0}

% Python style for highlighting
\newcommand\pythonstyle{\lstset{
language=Python,
basicstyle=\ttm,
otherkeywords={self},             % Add keywords here
keywordstyle=\ttb\color{deepblue},
emph={MyClass,__init__},          % Custom highlighting
emphstyle=\ttb\color{deepred},    % Custom highlighting style
stringstyle=\color{deepgreen},
frame=tb,                         % Any extra options here
showstringspaces=false            %
}}


% Python environment
\lstnewenvironment{python}[1][]
{
\pythonstyle
\lstset{#1}
}
{}

% Python for external files
\newcommand\pythonexternal[2][]{{
\pythonstyle
\lstinputlisting[#1]{#2}}}

% Python for inline
\newcommand\pythoninline[1]{{\pythonstyle\lstinline!#1!}}

\usepackage[colorlinks=true,linkcolor=blue]{hyperref}
\begin{document}
\tableofcontents

EOF




echo '\\newpage' >> $tex_file
echo '\section{object\\_movement.py}' >> $tex_file
echo '\pythonexternal{object_movement.py}' >>$tex_file

echo '\\newpage' >> $tex_file
echo "\section{cvlib.py}" >> $tex_file
echo "\pythonexternal{cvlib.py}" >>$tex_file


echo "\end{document}" >> $tex_file &&
pdflatex $tex_file -output-directory . &&
pdflatex $tex_file -output-directory .  # This needs to be run twice

                                           ## for the TOC to be generated
