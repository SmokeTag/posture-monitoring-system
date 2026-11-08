# Convenções do relatório final (para quem escreve um capítulo)

Arquivo principal: `report/relatorio_final.tex` (classe `article`, ABNT via
geometria/entrelinha). Cada capítulo é um `\input` de `report/final/NN_*.tex`.
**Edite apenas o(s) arquivo(s) do seu capítulo** (e os arquivos novos que o seu
briefing autorizar). Não faça commit — o integrador faz.

## Escrita
- Português do Brasil, registro acadêmico, impessoal ("mediu-se", "o sistema
  alerta"), sem marketing. Frases diretas; prefira números a adjetivos.
- **Honestidade é regra dura:** todo número tem que sair do repositório
  (docs/, data/README.md, analysis/, firmware/) ou de uma execução que você fez.
  Nada inventado. Onde faltar dado, use `\pendente{o que falta}` (macro já
  definida, sai em vermelho no PDF).
- Enquadramento do trabalho: o objetivo é **detectar postura prejudicial
  (alerta sim/não)**; classificar *qual* postura é bônus. Confusão em posturas
  combinadas (ex.: inclinar de lado e curvar ao mesmo tempo) é esperada, não bug.
- A feira de ciências (nov/2026) foi validação **informal** com dezenas de
  pessoas: alta taxa de acerto observada, **sem registro de dados**. Nunca
  apresentar como resultado quantitativo.
- Amostras pequenas: 8 sujeitos nas capturas de 21/10, 1 sujeito novo na
  validação de 22/10. Sem estatística inferencial.

## LaTeX
- Capítulo = `\section{}`, subdivisões `\subsection{}` / `\subsubsection{}` /
  `\paragraph{}`. Não redefina pacotes nem macros; se precisar de um pacote,
  diga no relatório final para o integrador adicionar (já há: graphicx,
  booktabs, tabularx, multirow, amsmath, gensymb, float, enumitem, xcolor,
  tikz [arrows.meta, positioning, fit, shapes.geometric, calc], hyperref,
  caption, setspace).
- Vírgula decimal: `10{,}5`; unidades com `\,`: `3{,}3\,k\ohm`, `20\,Hz`,
  `10\degree{}`, `\%` com `\,` antes.
- Figuras/tabelas no padrão ABNT: `\caption` **acima**, `\fonte{Elaborado
  pelo autor (2026)}` **abaixo**, posicionamento `[H]`. Tabelas com
  `booktabs` e dentro de `\begin{singlespace}`.
- Figuras PNG em `report/figures/` (referenciar só pelo nome do arquivo).
- Labels com prefixo do capítulo: `sec:met-*`, `fig:met-*`, `tab:met-*`,
  `eq:met-*` (metodologia); `res-` (resultados); `rev-` (revisão).
  Labels de capítulo já existentes: `sec:introducao`, `sec:revisao`,
  `sec:metodologia`, `sec:resultados`, `sec:discussao`, `sec:conclusao`.
- Citações no estilo autor-data manual já usado: "(OKEZUE et al., 2020)",
  "Segundo Iida e Guimarães (2016)". Referências novas: anote as entradas
  completas em ABNT NBR 6023 ao final do seu arquivo, num bloco de comentário
  `% REFERENCIAS NOVAS:` — o integrador move para `referencias.tex`.
- Nomes de código/arquivos em `\texttt{}`; use `\_` em nomes com sublinhado.
