/*
 * @file logicdata_csv_reader.h
 *
 * @date Jun 26, 2021
 * @author Andrey Belomutskiy, (c) 2012-2021
 */
class CsvReader {
public:
	CsvReader(size_t triggerCount) : CsvReader(triggerCount, 0.0) {}
	CsvReader(size_t triggerCount, double timestampOffset)
		: m_triggerCount(triggerCount)
		, m_timestampOffset(timestampOffset)
	{
	}

	void open(const char *fileName, const int* columnIndeces);
	bool haveMore();
	void processLine(EngineTestHelper *eth);
	void readLine(EngineTestHelper *eth);

	int lineIndex() const {
		return m_lineIndex;
	}

private:
	const size_t m_triggerCount;
	const double m_timestampOffset;

	FILE *fp;
	char buffer[255];

	bool currentState[2];

	int m_lineIndex = -1;

	const int* columnIndeces;
};

