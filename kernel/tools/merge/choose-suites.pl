#!/usr/bin/perl
# Chooses the test suites a merge's change can affect (unit 15J, README.md), from the paths it changes and the project
# graph in the .csproj files and Directory.Build.props, never a hand-kept list.
#
# Usage: perl choose-suites.pl [--rev <commit> [--base <commit>]] [--explain]
#   With no --rev: the change is the working tree and index against HEAD (a merge staged by merge-unit.sh).
#   With --rev: the change is <base>..<commit> (base: the commit's first parent), as a replay of a merge.
#   --explain also prints, for each test suite not chosen, the projects it builds on, none of which changed.
#
# Prints one line for each changed path and why it counts, then the choice, tab-separated:
#   ALL     <why>                         run every suite (dotnet test Kerf.slnx)
#   TEST    <project> <filter or -> <why>  run this test project (with this --filter, or all of it)
#   BUILD   <project> <why>               build this project (in the solution, but not a test project or under one)
#   NONE    <why>                         nothing to build or run
# Exits 5 if a kernel project (kernel/src, kernel/tests) reaches Kerf.Modeling: the kernel's suites must never depend
# on the modelling layer (ProjectGraphTests), or a change to the modelling layer alone could break them unseen.
use strict;
use warnings;

my ($rev, $base, $explain) = (undef, undef, 0);
while (@ARGV) {
    my $a = shift @ARGV;
    if ($a eq '--rev') { $rev = shift @ARGV; }
    elsif ($a eq '--base') { $base = shift @ARGV; }
    elsif ($a eq '--explain') { $explain = 1; }
    else { die "usage: choose-suites.pl [--rev <commit> [--base <commit>]] [--explain]\n"; }
}
$base //= defined $rev ? "$rev^1" : 'HEAD';

sub git { my $c = shift; my @o = `git $c`; die "git $c failed\n" if $?; chomp @o; s/\r$// for @o; return @o; }

# The tree at a commit, or the index with the working tree's contents when the commit is undefined
sub files_at { my $r = shift; return defined $r ? git("ls-tree -r --name-only $r") : git('ls-files --cached --others --exclude-standard'); }
my %read_cache;
sub read_at {
    my ($r, $p) = @_;
    my $k = ($r // q{}) . ":$p";
    return $read_cache{$k} //= read_now($r, $p);
}
sub read_now {
    my ($r, $p) = @_;
    if (defined $r) { my $t = `git show "$r:$p" 2>/dev/null`; return $? ? '' : $t; }
    open my $f, '<', $p or return '';
    local $/; my $t = <$f>; close $f; return $t;
}

my @changed = defined $rev ? git("diff --name-only --no-renames $base $rev")
    : (git("diff --name-only --no-renames $base"), git('ls-files --others --exclude-standard'));

# Normalises a path: forward slashes, no "." or ".." segments
sub norm {
    my $p = shift; $p =~ s{\\}{/}g;
    my @out;
    for my $s (split m{/}, $p) {
        next if $s eq '' || $s eq '.';
        if ($s eq '..') { pop @out; } else { push @out, $s; }
    }
    return join '/', @out;
}
sub dir_of { my $p = shift; return $p =~ m{^(.*)/[^/]*$} ? $1 : ''; }

# Resolves an Include written in a file at <file>, for a project at <proj>; undef if it names a property we don't know
sub resolve {
    my ($inc, $file, $proj) = @_;
    my $fdir = dir_of($file);
    my $root = ($proj =~ m{^([^/]+)/}) ? "$1/" : '';
    my %vars = (MSBuildThisFileDirectory => "$fdir/", MSBuildProjectDirectory => dir_of($proj) . '/',
                KerfModelingRoot => 'modeling/', KerfKernelRoot => 'kernel/', RepoRoot => $root);
    my $ok = 1;
    $inc =~ s/\$\((\w+)\)/exists $vars{$1} ? $vars{$1} : do { $ok = 0; '' }/ge;
    return undef unless $ok;
    my $abs = $inc =~ m{^(kernel|modeling)/} ? $inc : "$fdir/$inc";
    return norm($abs);
}
sub glob_re {
    my $g = quotemeta(shift);
    $g =~ s{\\\*\\\*/?}{\0}g; $g =~ s{\\\*}{[^/]*}g; $g =~ s{\0}{(?:.*/)?}g; $g =~ s{\\\?}{[^/]}g;
    return qr{^$g$}i;
}

# The project graph at one tree: projects, their props chains, references and the files they take from elsewhere
sub graph {
    my $r = shift;
    my @files = files_at($r);
    my %g = (projects => {}, refs => {}, items => {}, solution => {}, unresolved => []);
    my @projects = grep { /\.csproj$/ } @files;
    my %props = map { $_ => 1 } grep { m{(^|/)Directory\.Build\.(props|targets)$} } @files;
    for my $sln (grep { m{^[^/]+\.slnx$} } @files) {
        my $text = read_at($r, $sln);
        $g{solution}{norm($1)} = 1 while $text =~ /<Project Path="([^"]+)"/g;
    }
    for my $p (@projects) {
        $g{projects}{$p} = 1;
        # Directory.Build.props: the nearest above the project, and any it imports above that
        my @chain = ($p);
        my $d = dir_of($p);
        while (1) {
            my $found;
            for (my $e = $d; ; $e = dir_of($e)) {
                my $c = ($e eq '' ? '' : "$e/") . 'Directory.Build.props';
                if ($props{$c}) { $found = $c; last; }
                last if $e eq '';
            }
            last unless $found;
            push @chain, $found;
            last unless read_at($r, $found) =~ /GetPathOfFileAbove\('Directory\.Build\.props'/;
            $d = dir_of(dir_of($found));
            last if dir_of($found) eq '';
        }
        for my $file (@chain) {
            my $text = read_at($r, $file);
            $text =~ s/<!--.*?-->//gs;
            while ($text =~ m{<(ProjectReference|Compile|EmbeddedResource|None|Content|AdditionalFiles)\s+Include="([^"]+)"([^>]*?)(?:/>|>(.*?)</\1>)}gs) {
                my ($kind, $incs, $rest) = ($1, $2, ($3 // '') . ($4 // ''));
                # a None or Content item is an input only when copied beside the build (one only packed is not)
                next if ($kind eq 'None' || $kind eq 'Content') && $rest !~ /CopyTo(Output|Publish)Directory/;
                for my $inc (split /;/, $incs) {
                    my $res = resolve($inc, $file, $p);
                    if (!defined $res) {
                        push @{$g{unresolved}}, "$file: $inc" if $kind eq 'ProjectReference';
                        next;
                    }
                    if ($kind eq 'ProjectReference') { $g{refs}{$p}{$res} = 1 unless $res eq $p; }
                    else { push @{$g{items}{$p}}, [glob_re($res), $res, $file]; }
                }
            }
        }
    }
    return \%g;
}

my @graphs = (graph($base), graph($rev));
my (%projects, %refs, %items, %solution, @unresolved);
for my $g (@graphs) {
    $projects{$_} = 1 for keys %{$g->{projects}};
    $solution{$_} = 1 for keys %{$g->{solution}};
    for my $p (keys %{$g->{refs}}) { $refs{$p}{$_} = 1 for keys %{$g->{refs}{$p}}; }
    for my $p (keys %{$g->{items}}) { push @{$items{$p}}, @{$g->{items}{$p}}; }
    push @unresolved, @{$g->{unresolved}};
}
my %users;    # project -> the projects that reference it
for my $p (keys %refs) { $users{$_}{$p} = 1 for keys %{$refs{$p}}; }
sub closure {
    my ($edges, @start) = @_;
    my %seen; my @todo = @start;
    while (@todo) { my $q = shift @todo; next if $seen{$q}++; push @todo, keys %{$edges->{$q} // {}}; }
    return \%seen;
}
sub name { my $p = shift; $p =~ s{^.*/}{}; $p =~ s{\.csproj$}{}; return $p; }
sub is_test { return name(shift) =~ /\.Tests$/; }

# The rule: no kernel project reaches Kerf.Modeling, directly or through another project
my $modeling = 'modeling/src/Kerf.Modeling/Kerf.Modeling.csproj';
for my $p (sort grep { m{^kernel/(src|tests)/} } keys %projects) {
    if (closure(\%refs, $p)->{$modeling}) {
        print "RULE\t$p reaches Kerf.Modeling: the kernel's suites must not depend on the modelling layer\n";
        exit 5;
    }
}

# Each changed path: the projects it is an input of, or what kind of path it is
my %direct;           # changed project -> why
my (%filtered, @all, $tools);
my $api_test = 'FullyQualifiedName~PublicApiBaselineTests';
my %records = ('kernel/docs/api/public-api.txt' => 'kernel/tests/Kerf.Kernel.PublicApi.Tests/Kerf.Kernel.PublicApi.Tests.csproj',
               'modeling/docs/public-api.txt' => 'modeling/tests/Kerf.Modeling.Tests/Kerf.Modeling.Tests.csproj');
my $graph_test = 'kernel/tests/Kerf.Kernel.PublicApi.Tests/Kerf.Kernel.PublicApi.Tests.csproj';
for my $path (@changed) {
    my $why;
    # what nothing can place: the build's settings for every project, the solutions, the analyzers, OCCT's version
    if ($path =~ m{(^|/)(Directory\.Build\.(props|targets)|\.editorconfig|\.gitattributes|global\.json|nuget\.config|occt-version\.json)$}i
        || $path =~ m{\.slnx?$} || $path =~ m{^kernel/src/Kerf\.Analyzers/} || $path =~ m{^kernel/\.deps}) {
        push @all, $path; print "path\t$path\tall: the build's settings, a solution, the analyzers or OCCT's version\n"; next;
    }
    if (exists $records{$path}) {
        $filtered{$records{$path}} = $api_test; print "path\t$path\tthe public API's record: its check\n"; next;
    }
    # the projects whose own folder holds it (the deepest), or that take it from elsewhere (a link or a shared item)
    my @owners;
    my ($owner) = sort { length($b) <=> length($a) } grep { index($path, dir_of($_) . '/') == 0 } keys %projects;
    for my $p (keys %items) {
        next if defined $owner && $p eq $owner;
        for my $it (@{$items{$p}}) {
            # an item under the project's own folder is its own file, already placed by the folder
            next if index($it->[1], dir_of($p) . '/') == 0;
            if ($path =~ $it->[0]) { push @owners, $p; last; }
        }
    }
    my $doc = $path =~ m{\.md$}i || $path =~ m{(^|/)docs/} || $path =~ m{^(kernel|modeling)/(LICENSE|NOTICE|OCCT_LGPL_EXCEPTION\.txt|licenses/.*)$}
        || $path =~ m{(^|/)\.gitignore$};
    # a document is no project's input unless a project names it as an item (copied, embedded or compiled)
    my $named = defined $owner && grep { $path =~ $_->[0] } @{$items{$owner} // []};
    if (!@owners && $doc && !$named) { print "path\t$path\ta document: no suite\n"; next; }
    push @owners, $owner if defined $owner;
    if (@owners) {
        for my $p (@owners) {
            $direct{$p} //= $path;
            # a changed project file can change the graph, so the rule's own test runs too
            $filtered{$graph_test} //= 'FullyQualifiedName~ProjectGraphTests' if $path =~ /\.csproj$/;
        }
        print "path\t$path\tinput of ", join(', ', map { name($_) } sort @owners), "\n";
        next;
    }
    if ($path =~ m{^(kernel|modeling)/(tools|benchmarks)/} || $path =~ m{^modeling/tests/reference/}) {
        $tools = $path; print "path\t$path\ta tool or benchmark no project builds: the benchmarks' build\n"; next;
    }
    push @all, $path; print "path\t$path\tall: no rule places it\n";
}

if (@unresolved) { print "ALL\ta reference names a property this script doesn't know: $unresolved[0]\n"; exit 0; }
if (@all) { print "ALL\t$all[0]", (@all > 1 ? ' and ' . (@all - 1) . ' more' : ''), "\n"; exit 0; }

# Every project that builds on a changed one, through references; the tests among them run, the rest build
my $affected = closure(\%users, keys %direct);
my (@tests, @builds);
for my $p (sort keys %$affected) {
    next unless $solution{$p};
    push @tests, $p if is_test($p);
}
my $built = closure(\%refs, @tests);
for my $p (sort keys %$affected) {
    next unless $solution{$p} && !is_test($p) && !$built->{$p};
    push @builds, $p;
}
if (defined $tools) {
    for my $p (sort grep { m{^kernel/benchmarks/} && !is_test($_) } keys %solution) {
        push @builds, $p unless $built->{$p} || grep { $_ eq $p } @builds;
    }
}
# why each runs: the changed project it builds on (its own, or the first found)
sub because {
    my $p = shift;
    return "changed ($direct{$p})" if exists $direct{$p};
    my $c = closure(\%refs, $p);
    my ($d) = sort grep { $c->{$_} } keys %direct;
    return defined $d ? 'builds on ' . name($d) : 'a tool or benchmark changed';
}
my %chosen = map { $_ => 1 } @tests;
print "TEST\t$_\t-\t", because($_), "\n" for @tests;
for my $p (sort keys %filtered) {
    next if $chosen{$p};
    print "TEST\t$p\t$filtered{$p}\t", ($filtered{$p} =~ /Graph/ ? 'a project file changed: the rule on the graph' : 'its public API record changed'), "\n";
}
print "BUILD\t$_\t", because($_), "\n" for @builds;
print "NONE\tdocuments only\n" unless @tests || @builds || %filtered;

if ($explain) {
    for my $p (sort grep { $solution{$_} && is_test($_) && !$chosen{$_} } keys %projects) {
        my $c = closure(\%refs, $p);
        my @on = sort map { name($_) } grep { $_ ne $p } keys %$c;
        # checked, not assumed: neither it nor anything it builds on is an input of a changed path
        my @hit = grep { exists $direct{$_} } keys %$c;
        print "SKIP\t", name($p), "\tbuilds on ", (@on ? join(', ', @on) : 'nothing of ours'),
            (@hit ? "; BUT " . join(', ', map { name($_) } @hit) . " changed\n" : "; none changed\n");
    }
}
