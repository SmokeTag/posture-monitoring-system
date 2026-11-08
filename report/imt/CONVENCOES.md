# Convenções — versão IMT (modelo abnTeX2 do Instituto Mauá)

Versão de entrega do TCC no modelo da universidade. **Limite: 15 páginas de
texto** (Introdução → Conclusões, sem capa/resumo, abstract e referências).
A versão longa (59 p.), já revisada, está em `report/relatorio_final.tex` +
`report/final/*.tex` e é a **fonte** do conteúdo: aqui se condensa, não se
reescreve do zero. Todo número tem que existir na versão longa ou no repo.

## Orçamento de páginas (inclui figuras e tabelas)
| capítulo | arquivo | páginas |
|---|---|---|
| 1 Introdução (com objetivos) | `capitulos/1introducao.tex` | 1,5 |
| 2 Revisão Bibliográfica | `capitulos/2revisao_bibliografica.tex` | 3 |
| 3 Metodologia: 3.1 Colar · 3.2 Cadeira · 3.3 Integração | `capitulos/3metodologia.tex` | 5,5 |
| 4 Testes e Resultados (protocolos + resultados + discussão) | `capitulos/4resultados.tex` | 4 |
| 5 Conclusões e Trabalhos Futuros | `capitulos/5conclusao.tex` | 1 |

Medir: `python3 report/imt/paginas.py [outdir]` (compila com Tectonic —
`tectonic` no PATH ou variável `TECTONIC` — e mostra a página de início de
cada capítulo; capítulos não quebram página, então a contagem por capítulo é
aproximada — vale o total).

## Escrita
- PT-BR acadêmico, impessoal, 3ª pessoa. Frases curtas; números no lugar de
  adjetivos. Cortar repetição entre capítulos (cada fato num lugar só).
- Objetivo do sistema = **detectar** postura prejudicial (alerta sim/não);
  classificar é bônus. Feira (nov/2026) = qualitativa, sem números.
- Diagramas de blocos e fluxogramas (TikZ) são preferidos para explicar
  funcionamento e lógica de programa (pedido da orientadora).
- Nada de detalhes de implementação que não ajudem o leitor (nomes de
  campos do CSV, hashes de commit, flags de linha de comando, versões de
  protocolo) — salvo quando forem essenciais.
- Pendências: `\pendente{...}` (sai em vermelho).

## LaTeX (abnTeX2)
- Capítulo = `\chapter` (já criado com label); seções `\section`,
  `\subsection`. Sem `\subsubsection` se der.
- Figuras/tabelas: `\caption{}` **acima**, conteúdo, depois
  `\fonte{Elaborado pelos autores (2026)}` (comando do abnTeX2), depois
  `\label{}`. Posição `[H]` ou `[htbp]`. Citar toda figura no texto
  ("a Figura~\ref{...} apresenta", "a seguir" — nunca "abaixo/acima").
- Tabelas com `booktabs`; `\small`/`\footnotesize` se precisar.
- Pacotes disponíveis: amsmath, gensymb, tabularx, multirow, booktabs,
  xcolor, tikz (arrows.meta, positioning, fit, shapes.geometric, calc,
  backgrounds), float, subfig, graphicx. Não adicione pacotes.
- Unidades em texto, fora de `$...$`: `20\,Hz`, `3{,}3\,k\ohm`, `10\degree{}`,
  `5\,\%`. Vírgula decimal `{,}` em math.
- Figuras PNG: só o nome do arquivo (`res_alerta_sujeitos.png`); o
  `\graphicspath` acha `report/figures/` e `analysis/archive/figures/`.
- Labels com prefixo: `fig:met-`, `tab:res-`, `sec:rev-` etc.
- **Citações:** `\cite{chave}` → (SOBRENOME, ano); `\citeonline{chave}` →
  Sobrenome (ano). Chaves em `bibliografia.bib`:
  bourahmoune2019, breen2009, dul2012, ferreira2025, grandjean1998,
  haller2011, iida2016, interlink_datasheet, interlink_guia, iso11226,
  kappattanavar2021, kuo2021, kuorinka1987, luinge2005, mcatamney1993,
  meyer2010, mohammadian2025, mutlu2007, okezue2020, paredesmadrid2017,
  pedley2013, ribeiro2014, ribeiro2020, robertson2013, roh2018, sonne2012,
  tan2001, wong2008, xu2013, zemp2016, zheng2013.
  Componentes (se verificáveis): st_lsm6ds3trc, nordic_nrf52840,
  ti_cd74hc4067, seeed_xiao, nordic_esb, tmrh20_nrftonrf.
