#!/usr/bin/env python3
"""Compila main.tex e mostra em que página começa cada capítulo (limite: 15
páginas no total, da capa ao fim das Referências).

Uso: python3 report/imt/paginas.py [outdir]   (padrão: um diretório temporário)
Compilador: Tectonic (variável TECTONIC ou `tectonic` no PATH); precisa de
pdfinfo/pdftotext (poppler). Os capítulos não quebram página neste modelo,
então a contagem por capítulo é aproximada — vale o total."""
import os, re, shutil, subprocess, sys, tempfile

SRC = os.path.dirname(os.path.abspath(__file__))
tec = os.environ.get('TECTONIC') or shutil.which('tectonic')
if not tec:
    sys.exit('Tectonic não encontrado: instale ou defina TECTONIC=/caminho/tectonic')
out = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else tempfile.mkdtemp(prefix='tcc-imt-'))
os.makedirs(out, exist_ok=True)
r = subprocess.run([tec, '-X', 'compile', '--outdir', out, 'main.tex'], cwd=SRC,
                   capture_output=True, text=True)
errs = [l for l in (r.stdout + r.stderr).splitlines()
        if l.startswith('error') or 'Overfull' in l or 'undefined' in l.lower()]
print('\n'.join(sorted(set(errs))) or 'compilou sem erros/overfull')
pdf = os.path.join(out, 'main.pdf')
if r.returncode or not os.path.exists(pdf):
    sys.exit(1)
info = subprocess.run(['pdfinfo', pdf], capture_output=True, text=True).stdout
n = int(re.search(r'Pages:\s+(\d+)', info).group(1))
heads = ['1 Introdução', '2 Revisão', '3 Metodologia', '4 Testes e Resultados',
         '5 Conclusões', 'Referências']
start = {}
for p in range(3, n + 1):
    t = subprocess.run(['pdftotext', '-f', str(p), '-l', str(p), pdf, '-'],
                       capture_output=True, text=True).stdout
    for h in heads:
        if h not in start and re.search(r'^\s*' + re.escape(h), t, re.M):
            start[h] = p
print(f'PDF: {pdf} ({n} páginas)')
ks = [h for h in heads if h in start]
for a, b in zip(ks, ks[1:] + [None]):
    end = start[b] if b else n + 1
    print(f'  {a:25s} p.{start[a]:>3}  (~{end - start[a]} pág.)')
if '1 Introdução' in start and 'Referências' in start:
    print(f'TEXTO: ~{start["Referências"] - start["1 Introdução"]} páginas')
print(f'TOTAL: {n} páginas (limite 15)')
