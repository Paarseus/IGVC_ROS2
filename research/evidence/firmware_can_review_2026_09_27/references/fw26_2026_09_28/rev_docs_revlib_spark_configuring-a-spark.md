<!-- Source: https://docs.revrobotics.com/revlib/spark/configuring-a-spark.md (fetched 2026-09-28) -->
> For the complete documentation index, see [llms.txt](https://docs.revrobotics.com/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.revrobotics.com/revlib/spark/configuring-a-spark.md).

# Configuring a SPARK

This page will discuss information about configuration concepts specific to only SPARK MAX and SPARK Flex. For more information on general configuration in REVLib, see [this page](/revlib/configuring-devices.md).

## Configuration Classes

SPARK MAX and SPARK Flex each have their own configuration classes, `SparkMaxConfig` and  `SparkFlexConfig`. They are both derived from `SparkBaseConfig` which includes shared configurations between the two devices. Configurations specific to SPARK MAX or SPARK Flex live in their respective configuration class.

### API Documentation

For more information about what configurations and sub-configuration classes each class provides, refer to the links below:

| `SparkMaxConfig`  | [Java](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/sparkmaxconfig)  | [C++](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_max_config.html)  |
| ----------------- | ------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------- |
| `SparkFlexConfig` | [Java](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/sparkflexconfig) | [C++](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_flex_config.html) |
| `SparkBaseConfig` | [Java](https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/sparkbaseconfig) | [C++](https://codedocs.revrobotics.com/cpp/classrev_1_1spark_1_1_spark_base_config.html) |

## Persisting Parameters

Configuring a SPARK MAX and SPARK Flex differs from other devices in REVLib with the addition of the `persistMode` parameter in their `configure()` methods, which specifies whether the configuration settings applied to the device should be persisted between power cycles.

Persisting parameters involves saving them to the SPARK controller's memory, which is time-intensive and blocks communication with the device. To provide flexibility, this process is not automatic, as this behavior may be unnecessary or undesirable in some cases. Therefore, users must manually specify the persist mode, and to help avoid possible pitfalls, it is a required parameter.

### Use Cases

It is recommended to persist parameters during the initial configuration of the device at the start of your program to ensure that the controller retains its configuration in the event of a power cycle during operation e.g. due to a breaker trip or a brownout.

When making updates to the configuration mid-operation, it is generally recommend to not persist the applied configuration changes to avoid blocking the program, depending on the use case. While reconfiguring a device during operation is generally discouraged, some use cases may necessitate it, and it is important to make the choice whether to persist parameters as it can affect performance of the robot.

Below is an example of either case:

{% tabs %}
{% tab title="Java" %}

```java
Robot() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    // Persist parameters to retain configuration in the event of a power cycle
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    
    // Don't persist parameters since it takes time and this change is temporary
    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
}
```

{% endtab %}
{% endtabs %}

## Defining Motor Type

Motor type is the only configuration parameter that must be set outside of a configuration object, specifically through the constructor of the `SparkMax` and `SparkFlex` classes. This ensures that the user makes the conscious decision the specify type of motor is being driven, as driving a brushless motor in brushed mode can permanently damage the motor.

Below is an example of how configuring for different motor types would look like:

{% tabs %}
{% tab title="Java" %}

<pre class="language-java"><code class="lang-java"><strong>SparkMax neo = new SparkMax(1, MotorType.kBrushless);
</strong><strong>SparkMax cim = new SparkMax(2, MotorType.kBrushed);
</strong>
SparkMaxConfig cimConfig = new SparkMaxConfig();

// Configure primary encoder for brushed motor
cimConfig.encoder
    .countsPerRevolution(8192)
    .inverted(true);
    
cim.configure(cimConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
</code></pre>

{% endtab %}
{% endtabs %}
