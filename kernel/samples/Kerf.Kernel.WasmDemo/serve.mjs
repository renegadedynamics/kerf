// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// A static server for the WebAssembly demo (Node 18 or later, no packages):
//   node serve.mjs -root <published wwwroot> -data <folder of .brep files> [-port 8642] [-out report.json] [-exit]
//                  [-exclude a.brep,b.brep]
// It serves the page, the .brep files under data/ (with data/list.json listing them), and takes the page's ?auto=1
// report on POST /report, written to -out; with -exit it stops after the report.
import { createServer } from 'node:http';
import { readFile, readdir, stat, writeFile } from 'node:fs/promises';
import { extname, join, normalize, resolve } from 'node:path';

const args = Object.fromEntries(process.argv.slice(2).reduce((pairs, a, i, all) => {
  if (a.startsWith('-')) pairs.push([a.slice(1), all[i + 1] && !all[i + 1].startsWith('-') ? all[i + 1] : true]);
  return pairs;
}, []));
const root = resolve(args.root ?? 'wwwroot');
const data = args.data ? resolve(args.data) : null;
const port = Number(args.port ?? 8642);
const types = { '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.json': 'application/json',
                '.wasm': 'application/wasm', '.dat': 'application/octet-stream', '.brep': 'application/octet-stream' };

async function list() {
  if (!data) return [];
  const excluded = new Set(typeof args.exclude === 'string' ? args.exclude.split(',') : []);
  const names = (await readdir(data)).filter((n) => n.toLowerCase().endsWith('.brep') && !excluded.has(n)).sort();
  return Promise.all(names.map(async (name) => ({ name, size: (await stat(join(data, name))).size })));
}

function inside(base, path) {
  const full = normalize(join(base, path));
  return full.startsWith(base) ? full : null;
}

createServer(async (request, response) => {
  try {
    const url = new URL(request.url, 'http://localhost');
    const path = decodeURIComponent(url.pathname);
    if (request.method === 'POST' && path === '/log') {
      const chunks = [];
      for await (const chunk of request) chunks.push(chunk);
      console.log(`[page] ${Buffer.concat(chunks).toString('utf8')}`);
      response.writeHead(204).end();
      return;
    }
    if (request.method === 'POST' && path === '/report') {
      const chunks = [];
      for await (const chunk of request) chunks.push(chunk);
      const body = Buffer.concat(chunks).toString('utf8');
      await writeFile(args.out ?? 'report.json', body);
      response.writeHead(204).end();
      console.log(`report written to ${args.out ?? 'report.json'}`);
      if (args.exit) setTimeout(() => process.exit(0), 100);
      return;
    }
    let file;
    if (path === '/data/list.json') {
      response.writeHead(200, { 'Content-Type': 'application/json' }).end(JSON.stringify(await list()));
      return;
    } else if (path.startsWith('/data/') && data) {
      file = inside(data, path.slice('/data/'.length));
    } else {
      file = inside(root, path === '/' ? 'index.html' : path.slice(1));
    }
    if (!file) {
      response.writeHead(403).end();
      return;
    }
    const body = await readFile(file);
    response.writeHead(200, { 'Content-Type': types[extname(file)] ?? 'application/octet-stream', 'Cache-Control': 'no-store' }).end(body);
  } catch {
    response.writeHead(404).end();
  }
}).listen(port, () => console.log(`http://localhost:${port}/ serving ${root}${data ? ` and ${data} under data/` : ''}`));
