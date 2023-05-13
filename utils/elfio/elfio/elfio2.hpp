#ifndef ELFIO2_HPP
#define ELFIO2_HPP

#ifdef _MSC_VER
#pragma warning ( push )
#pragma warning(disable:4996)
#pragma warning(disable:4355)
#pragma warning(disable:4244)
#endif

#include <elfio/elfio.hpp>

namespace ELFIO
{

class elfio2;

class symbol
{
public:
    friend class elfio2;

    explicit symbol(elfio& elf) : m_elf(elf) { }

    std::string   get_name()  const { return m_name; }
    Elf64_Addr    get_value() const { return m_value; }
    Elf_Xword     get_size()  const { return m_size; }
    unsigned char get_bind()  const { return m_bind; }
    unsigned char get_type()  const { return m_type; }
    Elf_Half      get_index() const { return m_index; }
    unsigned char get_other() const { return m_other; }

    void set_value(Elf64_Addr value)    { m_value = value; save(); }
    void set_size(Elf_Xword size)       { m_size = size;   save(); }
    void set_type(unsigned char type)   { m_type = type;   save(); }
    void set_bind(unsigned char bind)   { m_bind = bind;   save(); }
    void set_other(unsigned char other) { m_other = other; save(); }

protected:
    void save()
    {
        symbol_section_accessor symbols(m_elf, m_section);
        symbols.set_symbol(m_symbol_index, m_value, m_size, m_bind,
            m_type, m_index, m_other);
    }

    elfio        &m_elf;
    section      *m_section;
    std::string   m_name;
    Elf64_Addr    m_value;
    Elf_Xword     m_size;
    unsigned char m_bind;
    unsigned char m_type;
    Elf_Half      m_index;
    unsigned char m_other;
    Elf_Xword     m_symbol_index;
};

class elfio2 : public elfio
{
public:
    elfio2() : elfio(), symbols(this) { }

    virtual ~elfio2()
    {
        for(std::vector<symbol*>::iterator it = m_symbols.begin();
            it != m_symbols.end(); ++it)
        {
            delete *it;
            *it = 0;
        }
    }

    const symbol* get_symbol_by_address(Elf64_Addr addr,
        bool allowLocal = false) const
    {
        const symbol *pSymbol = 0;

        for(std::vector<symbol*>::const_iterator it = this->m_symbols.begin();
            it != this->m_symbols.end()/* && !pSymbol*/; ++it)
        {
            Elf64_Addr startAddress = (*it)->get_value();
            Elf64_Addr endAddress = startAddress + (*it)->get_size();

            // Ignore local symbols if requested.
            if(!allowLocal && STB_LOCAL == (*it)->get_bind())
            {
                continue;
            }

            // For zero size sections, make sure the exact address will
            // trigger a match.
            if(startAddress == endAddress)
            {
                endAddress++;
            }

            if(addr >= startAddress && addr < endAddress)
            {
                pSymbol = *it;
            }
        }

        return pSymbol;
    }

    const section* get_section_by_address(Elf64_Addr addr) const
    {
        const section *pSection = 0;

        for(std::vector<section*>::const_iterator it = this->sections.begin();
            it != this->sections.end() && !pSection; ++it)
        {
            Elf64_Addr startAddress = (*it)->get_address();
            Elf64_Addr endAddress = startAddress + (*it)->get_size();

            if(addr >= startAddress && addr < endAddress)
            {
                pSection = *it;
            }
        }

        return pSection;
    }

    const section* get_section_by_address(Elf64_Addr addr,
        Elf_Word sectionType, Elf_Xword flags) const
    {
        const section *pSection = 0;

        for(std::vector<section*>::const_iterator it = this->sections.begin();
            it != this->sections.end() && !pSection; ++it)
        {
            Elf64_Addr startAddress = (*it)->get_address();
            Elf64_Addr endAddress = startAddress + (*it)->get_size();

            if(addr >= startAddress && addr < endAddress &&
                (*it)->get_type() == sectionType &&
                ((*it)->get_flags() & flags) == flags)
            {
                pSection = *it;
            }
        }

        return pSection;
    }

    std::vector<const section*> get_sections_by_address(Elf64_Addr addr) const
    {
        std::vector<const section*> sections_;

        for(std::vector<section*>::const_iterator it = this->sections.begin();
            it != this->sections.end(); ++it)
        {
            Elf64_Addr startAddress = (*it)->get_address();
            Elf64_Addr endAddress = startAddress + (*it)->get_size();

            if(addr >= startAddress && addr < endAddress)
            {
                sections_.push_back(*it);
            }
        }

        return sections_;
    }

    bool load(const std::string& file_name)
    {
        bool result = elfio::load(file_name);

        if(result)
        {
            result = load_symbol_table();
        }

        return result;
    }

    bool load_symbol_table()
    {
        for(std::vector<section*>::iterator it = this->sections.begin();
            it != this->sections.end(); ++it)
        {
            Elf_Word sectionType = (*it)->get_type();

            if(SHT_SYMTAB == sectionType || SHT_DYNSYM == sectionType)
            {
                symbol_section_accessor symbols(*this, *it);

                for(Elf_Xword i = 0; i < symbols.get_symbols_num(); ++i)
                {
                    symbol *pSymbol = new symbol(*this);
                    pSymbol->m_section = *it;
                    pSymbol->m_symbol_index = i;

                    if(!symbols.get_symbol(i, pSymbol->m_name, pSymbol->m_value,
                        pSymbol->m_size, pSymbol->m_bind, pSymbol->m_type,
                        pSymbol->m_index, pSymbol->m_other))
                    {
                        delete pSymbol;
                        pSymbol = 0;

                        return false;
                    }

                    m_symbols.push_back(pSymbol);
                }
            }
        }

        return true;
    }

    friend class Symbols;
    class Symbols
    {
    public:
        Symbols(elfio2 *parent_) : parent(parent_) { }

        Elf_Xword size() const
        {
            return (Elf_Xword)parent->m_symbols.size();
        }

        symbol* operator[](unsigned int index) const
        {
            symbol *pSymbol = 0;

            if(index < parent->m_symbols.size())
            {
                pSymbol = parent->m_symbols[index];
            }

            return pSymbol;
        }

        symbol* operator[](const std::string& name) const
        {
            symbol *pSymbol = 0;

            for(std::vector<symbol*>::const_iterator it =
                parent->m_symbols.begin(); it != parent->m_symbols.end() &&
                !pSymbol; ++it)
            {
                if((*it)->get_name() == name && STB_LOCAL != (*it)->get_bind())
                {
                    pSymbol = *it;
                }
            }

            return pSymbol;
        }

        std::vector<symbol*>::iterator begin()
        {
            return parent->m_symbols.begin();
        }

        std::vector<symbol*>::const_iterator begin() const
        {
            return parent->m_symbols.begin();
        }

        std::vector<symbol*>::iterator end()
        {
            return parent->m_symbols.end();
        }

        std::vector<symbol*>::const_iterator end() const
        {
            return parent->m_symbols.end();
        }
    private:
        elfio2 *parent;
    } symbols;

protected:
    std::vector<symbol*> m_symbols;
};

}; // namespace ELFIO

#ifdef _MSC_VER
#pragma warning ( pop )
#endif

#endif // ELFIO2_HPP
