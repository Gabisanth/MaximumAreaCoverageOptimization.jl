module MaxAreaOptimization

function optimize(input, obj, cons_ext, cons_prog, N_iter)

    # Define optimization problem
    global p = DSProblem(length(input))
    SetInitialPoint(p, input)
    SetObjective(p, obj)
    SetIterationLimit(p, N_iter)


    # Add constraints to problem
    for i in cons_ext
        AddExtremeConstraint(p, i)
    end
    for i in cons_prog
        AddProgressiveConstraint(p, i)
    end

    Optimize!(p)

    if p.x === nothing #tests object identity.
        global result = p.i
    else
        global result = p.x
    end

    ReportStatus(p)
    return result

end


end