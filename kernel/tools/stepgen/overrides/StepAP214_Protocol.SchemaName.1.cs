    public override string SchemaName(Interface_InterfaceModel theModel)
    {
        StepData_StepModel aModel1 = theModel as StepData_StepModel;
        DESTEP_Parameters.WriteMode_StepSchema aSchema = aModel1 is null ? DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP214IS
                                                                          : aModel1.InternalParameters.WriteSchema;
        switch (aSchema)
        {
            case DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203:
                return schemaAP203;
            case DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP214IS:
                return schemaAP214IS;
            case DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP242DIS:
                return schemaAP242DIS;
            case DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP214CD:
                return schemaAP214CD;
            case DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP214DIS:
            default:
                return schemaAP214DIS;
        }
    }
