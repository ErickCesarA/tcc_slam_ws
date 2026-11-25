#!/usr/bin/env python3
"""Exporta relatórios HTML locais do TCC SLAM para PDF A4.

Uso:
    python3 ~/tcc_slam_ws/exportar_pdf.py /caminho/relatorio.html
    python3 ~/tcc_slam_ws/exportar_pdf.py --todos ~/tcc_slam_ws/results

O HTML e os PNGs originais nao sao alterados. Nao requer ROS 2 nem Gazebo.
"""

import argparse
from pathlib import Path
import sys


CSS_IMPRESSAO = """
@page { size: A4; margin: 12mm; }
@media print {
    html, body { background: white !important; }
    header { padding: 10mm 6mm !important; }
    main { max-width: none !important; padding: 6mm 0 !important; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .mapas { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .scroller { overflow: visible !important; }
    table { font-size: 9pt !important; width: 100% !important; table-layout: fixed; }
    th, td { padding: 5px !important; overflow-wrap: anywhere; }
    h1, h2, h3 { break-after: avoid; }
    .card, article, section { break-inside: avoid; }
    img { max-width: 100% !important; height: auto !important; }
    section img { max-height: 230mm; object-fit: contain; }
}
"""


def exportar(html: Path, destino: Path | None = None) -> Path:
    try:
        from weasyprint import HTML, CSS
    except ImportError as exc:
        raise RuntimeError(
            "WeasyPrint nao instalado. No Ubuntu, tente: "
            "sudo apt install python3-weasyprint"
        ) from exc

    html = html.expanduser().resolve()
    if not html.is_file() or html.suffix.lower() != '.html':
        raise ValueError(f'Arquivo HTML invalido: {html}')

    pdf = destino.expanduser().resolve() if destino else html.with_suffix('.pdf')
    pdf.parent.mkdir(parents=True, exist_ok=True)

    # filename e base_url fazem os graficos relativos carregarem no PDF.
    HTML(filename=str(html)).write_pdf(
        str(pdf), stylesheets=[CSS(string=CSS_IMPRESSAO)]
    )
    if not pdf.is_file() or pdf.stat().st_size < 1000:
        raise RuntimeError(f'PDF nao foi criado corretamente: {pdf}')
    print(f'PDF gerado: {pdf} ({pdf.stat().st_size / 1024:.1f} KiB)')
    return pdf


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Converte relatorios HTML de experimentos SLAM em PDF.'
    )
    parser.add_argument('caminho', type=Path, help='Arquivo HTML, ou pasta de resultados com --todos')
    parser.add_argument('--todos', action='store_true', help='Procura relatorios recursivamente')
    parser.add_argument('--forcar', action='store_true', help='Regenera PDFs ja atualizados')
    args = parser.parse_args()

    if args.todos:
        raiz = args.caminho.expanduser().resolve()
        if not raiz.is_dir():
            parser.error(f'Diretorio nao encontrado: {raiz}')
        arquivos = sorted(
            p for p in raiz.rglob('*.html')
            if p.name in ('relatorio.html', 'relatorio_comparativo.html')
        )
        if not arquivos:
            parser.error(f'Nenhum relatorio encontrado em: {raiz}')
    else:
        arquivos = [args.caminho]

    erros = 0
    for html in arquivos:
        html = html.expanduser().resolve()
        if args.todos and not args.forcar and html.with_suffix('.pdf').exists():
            pdf = html.with_suffix('.pdf')
            if pdf.stat().st_mtime >= html.stat().st_mtime:
                print(f'Atualizado: {pdf}')
                continue
        try:
            exportar(html)
        except (OSError, ValueError, RuntimeError) as exc:
            print(f'ERRO: {exc}', file=sys.stderr)
            erros += 1
    print(f'Arquivos HTML: {len(arquivos)} | falhas: {erros}')
    return 1 if erros else 0


if __name__ == '__main__':
    raise SystemExit(main())
