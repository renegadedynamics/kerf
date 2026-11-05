// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_EntityReducer.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Tidy;

/// <summary>
/// The base of the reducers: finds the entities of a type equal to one seen before, and replaces each with the first
/// wherever it's referred to. It only replaces: the duplicates stay in the model until
/// <see cref="DuplicateCleaner"/> removes them. An entity is left alone if anything refers to it through a type with no
/// replacer.
/// </summary>
/// <typeparam name="TEntity">The type of the entities processed.</typeparam>
internal abstract class EntityReducer<TEntity>(ExchangeWorkSession workSession, IEqualityComparer<TEntity> hasher)
    where TEntity : Transient
{
    /// <summary>Replaces an old entity with a new one in a sharing entity; false if it held no such reference.</summary>
    protected delegate bool Replacer(TEntity oldEntity, TEntity newEntity, Transient sharing);

    private readonly Dictionary<Type, Replacer> _replacers = [];
    private readonly Dictionary<TEntity, List<TEntity>> _duplicates = new(hasher);

    /// <summary>
    /// Takes an entity into account if it's of the type, and every entity referring to it has a replacer.
    /// </summary>
    /// <returns>Whether the entity was of the type.</returns>
    public bool ProcessEntity(Transient entity)
    {
        if (entity is not TEntity typed)
        {
            return false;
        }

        if (HasAllReplacers(workSession.Graph().GetSharings(typed)))
        {
            if (_duplicates.TryGetValue(typed, out List<TEntity>? list))
            {
                list.Add(typed);
            }
            else
            {
                _duplicates.Add(typed, []);
            }
        }

        return true;
    }

    /// <summary>
    /// Replaces each duplicate with the first entity equal to it, and adds those fully replaced to
    /// <paramref name="replacedEntities"/>.
    /// </summary>
    public void Perform(HashSet<Transient> replacedEntities)
    {
        EntityGraph graph = workSession.Graph();
        foreach ((TEntity entity, List<TEntity> duplicates) in _duplicates)
        {
            foreach (TEntity duplicate in duplicates)
            {
                NCollection_HSequence<Transient>? sharings = graph.GetSharings(duplicate);
                if (sharings is not null && ReplaceInSharings(duplicate, entity, sharings))
                {
                    replacedEntities.Add(duplicate);
                }
            }
        }
    }

    /// <summary>Registers the replacer for a type of sharing entity, before any entity is processed.</summary>
    protected void RegisterReplacer(Type type, Replacer replacer) => _replacers[type] = replacer;

    private bool HasAllReplacers(NCollection_HSequence<Transient>? sharings)
    {
        if (sharings is null)
        {
            return false;
        }

        for (int i = 1; i <= sharings.Length(); i++)
        {
            if (!_replacers.ContainsKey(sharings.Value(i).GetType()))
            {
                return false;
            }
        }

        return true;
    }

    private bool ReplaceInSharings(TEntity oldEntity, TEntity newEntity, NCollection_HSequence<Transient> sharings)
    {
        bool isAllReplaced = true;
        for (int i = 1; i <= sharings.Length(); i++)
        {
            Transient sharing = sharings.Value(i);
            if (sharing is not null && !_replacers[sharing.GetType()](oldEntity, newEntity, sharing))
            {
                isAllReplaced = false;
            }
        }

        return isAllReplaced;
    }
}
