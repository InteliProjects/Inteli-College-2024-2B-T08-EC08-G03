import { Injectable, HttpException, HttpStatus } from '@nestjs/common';
import { SupabaseService } from './supabase.service';
import { User } from '../interfaces/user.interface';
import { CreateUser, UpdateUser } from 'src/dto/user.dto';

@Injectable()
export class UsersService {
  constructor(private readonly supabaseService: SupabaseService) {}

  async createUser(createUserDto: CreateUser) {
    const supabase = this.supabaseService.getClient();

    try {
      const { name, password, email } = createUserDto;
      const userData = { name, password, email };

      await supabase.from('user').insert(userData);

      return {
        status: 'success',
        message: 'Usuário criado com sucesso',
      };
    } catch (error) {
      console.error('Erro ao criar usuário:', error);
      throw new HttpException(
        `Erro ao criar usuário: ${error.message}`,
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }

  async listUsers(): Promise<User[]> {
    const supabase = this.supabaseService.getClient();

    try {
      const { data } = await supabase.from('user').select('*');

      return data as User[];
    } catch (error) {
      throw new HttpException(
        `Erro ao listar usuários: ${error.message}`,
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }

  async getUser(id: number): Promise<User> {
    const supabase = this.supabaseService.getClient();

    try {
      const { data } = await supabase.from('user').select('*').eq('id', id);

      return data[0];
    } catch (error) {
      console.error('Erro ao buscar usuário:', error);
      throw new HttpException(
        `Erro ao buscar usuário: ${error.message}`,
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }

  async updateUser(id: number, updateUserDto: UpdateUser): Promise<{ message: string }> {
    const supabase = this.supabaseService.getClient();

    try {
      const { name, password, email } = updateUserDto;
      await supabase
        .from('user')
        .update({ name, password, email })
        .eq('id', id);

      return { message: 'Usuário atualizado com sucesso' };
    } catch (error) {
      console.error('Erro ao atualizar usuário:', error);
      throw new HttpException(
        `Erro ao atualizar usuário: ${error.message}`,
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }

  async deleteUser(id: number): Promise<{ message: string }> {
    const supabase = this.supabaseService.getClient();

    try {
      await supabase.from('user').delete().eq('id', id);

      return { message: 'Usuário deletado com sucesso' };
    } catch (error) {
      console.error('Erro ao deletar usuário:', error);
      throw new HttpException(
        `Erro ao deletar usuário: ${error.message}`,
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }
}
