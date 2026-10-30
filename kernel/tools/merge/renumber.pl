#!/usr/bin/perl
# Renumbers placeholder decisions (numbers >= 1000, written by unit branches) in docs/decisions.md to continue
# after the highest real number, in file order, and rewrites "decision N" / "decisions N" references to them in
# docs, src and tests. Usage: perl renumber.pl <repo root>
use strict;
use warnings;
use File::Find;

my $root = shift or die "usage: renumber.pl <repo> [floor]\n";
# Placeholders are numbered after the larger of this branch's highest number and the floor, so a branch can number
# past another integration branch's decisions
my $floor = shift // 0;
my $file = "$root/docs/decisions.md";
open my $in, '<', $file or die $!;
my @lines = <$in>;
close $in;

my $max = 0;
for (@lines) { $max = $1 if /^(\d+)\. / && $1 < 1000 && $1 > $max; }
$max = $floor if $floor > $max;

my %map;
my $next = $max + 1;
for (@lines) {
    if (/^(\d+)\. / && $1 >= 1000 && !exists $map{$1}) { $map{$1} = $next++; }
}

# Earlier renumberings, latest last: a reference to a placeholder that has no heading in this merge (a unit citing
# one renumbered before it merged) takes its most recent mapping. Placeholders are reused across units, so a
# heading in this merge always wins.
(my $histfile = __FILE__) =~ s/[^\/\\]+$/decision-map.txt/;
my %hist;
if (open my $hh, '<', $histfile) { while (<$hh>) { $hist{$1} = $2 if /^(\d+) (\d+)$/; } close $hh; }
my %new = %map;
for my $k (keys %hist) { $map{$k} = $hist{$k} unless exists $map{$k}; }
if (%new) { open my $ha, '>>', $histfile or die $!; print $ha "$_ $new{$_}\n" for sort { $a <=> $b } keys %new; close $ha; }
exit 0 unless %map;

# Every number in a reference list: "decision N", "decisions N, M and P–Q"
my $refs = sub {
    my ($t) = @_;
    $t =~ s{\b([Dd]ecisions?\s+\d+(?:\s*(?:,|and|–|-|to)\s*\d+)*)}{
        my $l = $1; $l =~ s/(\d+)/exists $map{$1} ? $map{$1} : $1/ge; $l
    }ge;
    return $t;
};

my $rewrite = sub {
    my ($text) = @_;
    $text =~ s/^(\d+)\. /exists $map{$1} ? "$map{$1}. " : "$1. "/gme;
    return $refs->($text);
};

my @files;
find(sub { push @files, $File::Find::name if -f && /\.(md|cs|csproj|props|targets|ps1|py|cpp|sh|sed)$/ && $File::Find::name !~ m{/(bin|obj|node_modules)/} },
     grep { -d } map { "$root/$_" } qw(docs src tests tools samples benchmarks));
for my $f (@files) {
    local $/;
    open my $h, '<', $f or next;
    my $old = <$h>;
    close $h;
    my $new = $f eq $file ? $rewrite->($old) : $refs->($old);
    if ($new ne $old) { open my $o, '>', $f or die $!; print $o $new; close $o; }
}
print join(", ", map { "$_->$new{$_}" } sort { $a <=> $b } keys %new), "\n";
